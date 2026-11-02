#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <VL53L1X.h>
#include <math.h>

// Wi-Fi hotspot
const char* AP_SSID = "Altivio-001";
const char* AP_PASSWORD = "altivio1234";

constexpr uint8_t SDA_PIN = 13;
constexpr uint8_t SCL_PIN = 14;
constexpr uint8_t BUTTON_PIN = 27;
constexpr uint8_t LED_PIN = 26;
constexpr uint8_t BUZZER_PIN = 25;

// Button settings
constexpr uint32_t CALIBRATION_HOLD_MS = 2000;
constexpr uint32_t DEBOUNCE_MS = 35;

// Sensor settings
constexpr uint8_t WINDOW_SAMPLES = 21;
constexpr uint8_t DISCARD_INITIAL_SAMPLES = 5;
constexpr uint16_t MIN_DISTANCE_MM = 50;
constexpr uint32_t WINDOW_TIMEOUT_MS = 5000;

constexpr float MAX_WINDOW_SD_MM = 5.5f;
constexpr uint16_t MAX_WINDOW_DIFFERENCE_MM = 8;
constexpr uint8_t ACQUISITION_RETRIES = 2;
constexpr uint16_t MAX_DISTANCE_MM = 4000;

// History
constexpr int HISTORY_SIZE = 100;

// Objects
VL53L1X sensor;
WebServer server(80);
Preferences prefs;

// Device state
bool sensorOK = false;
bool calibrated = false;
bool apStarted = false;

uint16_t floorReference = 0;
uint16_t lastHeadDistance = 0;
int offsetMM = 0;
float lastHeightMM = -1.0f;

String lastAction = "Ready";

// Button state
bool buttonStable = HIGH;
bool buttonPrevious = HIGH;
bool longPressTriggered = false;

uint32_t lastButtonChange = 0;
uint32_t buttonPressedAt = 0;

// Measurement history
struct Measurement {
  uint32_t timeMs;
  uint16_t distanceMM;
  float heightMM;
};

Measurement history[HISTORY_SIZE];
int historyCount = 0;
int historyNext = 0;

void setLED(bool on) {
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

void beep(uint16_t durationMs = 100) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(durationMs);
  digitalWrite(BUZZER_PIN, LOW);
}

void successBeep() {
  beep(150);
}

void errorBeep() {
  beep(400);
}

void sortDistances(uint16_t *values, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) {
    uint16_t value = values[i];
    int j = i - 1;
    while (j >= 0 && values[j] > value) {
      values[j + 1] = values[j];
      j--;
    }
    values[j + 1] = value;
  }
}

uint16_t medianOfSorted(const uint16_t *values, uint8_t n) {
  if (n == 0) return 0;
  if (n % 2) return values[n / 2];
  return (uint16_t)((values[n / 2 - 1] + values[n / 2]) / 2);
}

bool collectMeasurementWindow(uint16_t &result, uint8_t &keptOut,
                              float &sdOut, float &madOut) {
  uint16_t samples[WINDOW_SAMPLES];
  uint8_t count = 0;
  uint8_t discard = DISCARD_INITIAL_SAMPLES;
  uint32_t started = millis();
  uint16_t invalid = 0;

  while (count < WINDOW_SAMPLES &&
         (uint32_t)(millis() - started) < WINDOW_TIMEOUT_MS) {
    if (!sensor.dataReady()) {
      if (sensor.last_status != 0) {
        Serial.println("Sensor I2C communication error while waiting for data.");
        return false;
      }
      delay(1);
      continue;
    }

    uint16_t distance = sensor.read(false);
    bool timedOut = sensor.timeoutOccurred();

    if (sensor.last_status != 0) {
      Serial.println("Sensor I2C read error.");
      return false;
    }

    if (discard > 0) {
      discard--;
      continue;
    }

    if (timedOut ||
        sensor.ranging_data.range_status != VL53L1X::RangeValid ||
        distance < MIN_DISTANCE_MM || distance > MAX_DISTANCE_MM) {
      invalid++;
      continue;
    }

    samples[count++] = distance;
  }

  if (count != WINDOW_SAMPLES) {
    Serial.printf("Window timeout: got %u/%u valid samples; invalid=%u\n",
                  count, WINDOW_SAMPLES, invalid);
    return false;
  }

  uint16_t sorted[WINDOW_SAMPLES];
  uint16_t deviations[WINDOW_SAMPLES];
  bool inlier[WINDOW_SAMPLES];

  for (uint8_t i = 0; i < WINDOW_SAMPLES; i++) sorted[i] = samples[i];
  sortDistances(sorted, WINDOW_SAMPLES);
  uint16_t median = medianOfSorted(sorted, WINDOW_SAMPLES);

  for (uint8_t i = 0; i < WINDOW_SAMPLES; i++) {
    int delta = (int)samples[i] - (int)median;
    deviations[i] = (uint16_t)(delta < 0 ? -delta : delta);
  }
  sortDistances(deviations, WINDOW_SAMPLES);
  madOut = (float)medianOfSorted(deviations, WINDOW_SAMPLES);

  // MAD-based outlier limit. A 4 mm minimum accommodates integer-mm
  // quantisation; 10 mm cap prevents an unusually noisy window widening it.
  float cutoff = 3.0f * 1.4826f * madOut;
  if (cutoff < 4.0f) cutoff = 4.0f;
  if (cutoff > 10.0f) cutoff = 10.0f;

  float sum = 0.0f;
  keptOut = 0;
  for (uint8_t i = 0; i < WINDOW_SAMPLES; i++) {
    inlier[i] = fabsf((float)samples[i] - (float)median) <= cutoff;
    if (inlier[i]) {
      sum += samples[i];
      keptOut++;
    }
  }

  // Require at least 15 of 21 samples to agree with the median.
  if (keptOut < 15) {
    Serial.printf("Unstable window: only %u/%u samples retained.\n",
                  keptOut, WINDOW_SAMPLES);
    return false;
  }

  float mean = sum / keptOut;
  float varianceSum = 0.0f;
  for (uint8_t i = 0; i < WINDOW_SAMPLES; i++) {
    if (inlier[i]) {
      float delta = (float)samples[i] - mean;
      varianceSum += delta * delta;
    }
  }
  sdOut = sqrtf(varianceSum / (keptOut - 1));

  if (sdOut > MAX_WINDOW_SD_MM) {
    Serial.printf("Unstable window: sample SD %.2f mm exceeds %.2f mm.\n",
                  sdOut, MAX_WINDOW_SD_MM);
    return false;
  }

  result = (uint16_t)lroundf(mean);
  Serial.printf("Window result=%u mm; kept=%u/%u; median=%u; MAD=%.1f; SD=%.2f mm\n",
                result, keptOut, WINDOW_SAMPLES, median, madOut, sdOut);
  return true;
}

// One complete attempt: two independent robust windows must agree.
bool readStableDistanceOnce(uint16_t &finalResult) {
  uint16_t first = 0, second = 0;
  uint8_t kept1 = 0, kept2 = 0;
  float sd1 = 0, sd2 = 0, mad1 = 0, mad2 = 0;

  Serial.println("Collecting first independent sample window...");
  if (!collectMeasurementWindow(first, kept1, sd1, mad1)) return false;

  delay(150);

  Serial.println("Collecting second independent sample window...");
  if (!collectMeasurementWindow(second, kept2, sd2, mad2)) return false;

  uint16_t difference = (first > second) ? (first - second) : (second - first);
  Serial.printf("Window comparison: %u mm vs %u mm; difference=%u mm\n",
                first, second, difference);

  if (difference > MAX_WINDOW_DIFFERENCE_MM) {
    Serial.printf("Window pair rejected: difference %u mm exceeds %u mm.\n",
                  difference, MAX_WINDOW_DIFFERENCE_MM);
    return false;
  }

  finalResult = (uint16_t)lroundf(((float)first + (float)second) / 2.0f);
  Serial.printf("Accepted distance: %u mm\n", finalResult);
  return true;
}

uint16_t readStableDistance() {
  if (!sensorOK) {
    Serial.println("Cannot measure: VL53L1X is not ready.");
    return 0;
  }

  for (uint8_t attempt = 1; attempt <= ACQUISITION_RETRIES; attempt++) {
    Serial.printf("Measurement acquisition attempt %u/%u\n",
                  attempt, ACQUISITION_RETRIES);
    uint16_t result = 0;
    if (readStableDistanceOnce(result)) return result;

    if (attempt < ACQUISITION_RETRIES) {
      Serial.println("Retrying after a short settling pause...");
      delay(250);
    }
  }

  Serial.println("Measurement rejected after all acquisition attempts.");
  return 0;
}

void addHistory(uint16_t distance, float height) {
  history[historyNext].timeMs = millis();
  history[historyNext].distanceMM = distance;
  history[historyNext].heightMM = height;

  historyNext = (historyNext + 1) % HISTORY_SIZE;

  if (historyCount < HISTORY_SIZE) {
    historyCount++;
  }
}

bool calibrateFloor() {
  lastAction = "Calibrating floor";
  Serial.println("Floor calibration: keep the beam clear and sensor fixed.");
  setLED(true);

  lastHeadDistance = 0;
  lastHeightMM = -1.0f;

  if (!sensorOK) {
    lastAction = "Calibration failed: sensor unavailable";
    Serial.println(lastAction);
    setLED(false);
    errorBeep();
    return false;
  }

  uint16_t distance = readStableDistance();
  if (distance == 0) {
    lastAction = calibrated
      ? "Calibration failed after retries; previous reference retained"
      : "Calibration failed after retries; check sensor and Serial Monitor";
    Serial.println(lastAction);
    setLED(false);
    errorBeep();
    return false;
  }

  bool saved = prefs.putBool("calibrated", false) == 1;
  if (saved) saved = prefs.putUShort("floor", distance) == sizeof(uint16_t);
  if (saved) saved = prefs.putBool("calibrated", true) == 1;
  if (!saved) {
    calibrated = false;
    lastAction = "Calibration failed: could not save; please calibrate again";
    Serial.println(lastAction);
    setLED(false);
    errorBeep();
    return false;
  }

  floorReference = distance;
  calibrated = true;
  lastAction = "Floor calibrated successfully";
  Serial.printf("Floor reference saved: %u mm\n", (unsigned)floorReference);
  setLED(false);
  successBeep();
  return true;
}

bool measureHeight() {
  lastAction = "Measuring height";
  Serial.println("Starting height measurement...");
  lastHeadDistance = 0;
  lastHeightMM = -1.0f;

  if (!sensorOK) {
    lastAction = "Measurement failed: sensor unavailable";
    Serial.println(lastAction);
    errorBeep();
    return false;
  }
  if (!calibrated || floorReference < MIN_DISTANCE_MM ||
      floorReference > MAX_DISTANCE_MM) {
    lastAction = "Please calibrate the floor first";
    Serial.println(lastAction);
    errorBeep();
    return false;
  }

  setLED(true);
  uint16_t distance = readStableDistance();
  if (distance == 0) {
    lastAction = "Measurement failed after retries; keep target still and retry";
    Serial.println(lastAction);
    setLED(false);
    errorBeep();
    return false;
  }

  int rawHeight = (int)floorReference - (int)distance;
  if (rawHeight < -30) {
    lastAction = "Measurement failed: distance exceeds floor reference; recalibrate";
    Serial.println(lastAction);
    setLED(false);
    errorBeep();
    return false;
  }

  float height = (float)rawHeight + (float)offsetMM;
  if (height < 0) height = 0;
  lastHeadDistance = distance;
  lastHeightMM = height;
  addHistory(lastHeadDistance, lastHeightMM);
  lastAction = "Measurement completed";

  Serial.printf("Floor reference: %u mm\n", (unsigned)floorReference);
  Serial.printf("Head distance: %u mm\n", (unsigned)lastHeadDistance);
  Serial.printf("Height: %.1f cm\n", lastHeightMM / 10.0f);
  Serial.printf("Offset: %d mm\n", offsetMM);
  setLED(false);
  successBeep();
  return true;
}

void handleButton() {
  bool reading = digitalRead(BUTTON_PIN);
  uint32_t now = millis();

  if (reading != buttonPrevious) {
    buttonPrevious = reading;
    lastButtonChange = now;
  }

  // Debounce button state changes.
  if (now - lastButtonChange >= DEBOUNCE_MS &&
      reading != buttonStable) {

    buttonStable = reading;

    if (buttonStable == LOW) {
      // Button pressed.
      buttonPressedAt = now;
      longPressTriggered = false;

      Serial.println("Button pressed");
    } else {
      // Button released.
      uint32_t duration = now - buttonPressedAt;

      if (!longPressTriggered &&
          duration < CALIBRATION_HOLD_MS) {
        Serial.println("Short press: measurement");
        measureHeight();
      }

      longPressTriggered = false;
    }
  }

  // Trigger calibration while the button is still held.
  if (buttonStable == LOW &&
      !longPressTriggered &&
      now - buttonPressedAt >= CALIBRATION_HOLD_MS) {

    longPressTriggered = true;

    Serial.println("2-second hold reached: calibrating now");
    calibrateFloor();
  }
}

// =====================================================
// WEBPAGE
// =====================================================

const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Altivio Digital Height Meter</title>
<style>
* { box-sizing: border-box; }
body {
  font-family: Arial, sans-serif;
  background: #f3f5f7;
  color: #17202a;
  margin: 0 auto;
  padding: 16px;
  max-width: 760px;
}
.card {
  background: white;
  padding: 18px;
  margin: 14px 0;
  border-radius: 12px;
  box-shadow: 0 2px 8px #0001;
}
.big {
  font-size: 2.5rem;
  font-weight: bold;
  overflow-wrap: anywhere;
}
button, a {
  padding: 11px 14px;
  margin: 4px 2px;
  border: 0;
  border-radius: 7px;
  background: #176b4d;
  color: white;
  text-decoration: none;
  cursor: pointer;
}
button:disabled { opacity: .6; }
input, select {
  width: 100%;
  padding: 11px;
  margin: 7px 0 12px;
  border: 1px solid #bbb;
  border-radius: 7px;
  font-size: 1rem;
}
label { font-weight: bold; }
small { color: #555; }
.status { overflow-wrap: anywhere; }
table { width: 100%; border-collapse: collapse; }
td, th {
  padding: 8px;
  border-bottom: 1px solid #ddd;
  text-align: left;
}
.table-wrap { overflow-x: auto; }
.actions { display: flex; flex-wrap: wrap; gap: 5px; }
.primary { background: #176b4d; }
.secondary { background: #364152; }
</style>
</head>
<body>

<h2>Altivio Digital Height Meter</h2>

<div class="card">
  <small>Measured height</small>
  <div class="big" id="height">--</div>

  <label for="unit">Display unit</label>
  <select id="unit" onchange="refresh()">
    <option value="mm">Millimetres (mm)</option>
    <option value="cm" selected>Centimetres (cm)</option>
    <option value="in">Inches (in)</option>
    <option value="ft">Feet + inches (ft / in)</option>
  </select>

  <p>Head distance: <b id="distance">--</b></p>
  <p>Floor reference: <b id="floor">--</b></p>
  <p>Correction offset: <b id="offsetDisplay">--</b></p>
  <p>Sensor: <b id="sensor">--</b></p>
  <p>Calibration: <b id="calibration">--</b></p>
  <p class="status">Last action: <b id="action">--</b></p>
</div>

<div class="card">
  <h3>Device controls</h3>

  <div class="actions">
    <button id="measureBtn" class="primary"
      onclick="runAction('/measure')">Measure height</button>

    <button id="calibrateBtn" class="secondary"
      onclick="runAction('/calibrate')">Calibrate floor</button>
  </div>

  <p><small>
    The physical button also works: release before 2 seconds
    for measurement; keep holding for 2 seconds to begin
    calibration automatically. Each result uses two independent
    filtered sample windows.
  </small></p>

  <p id="message" role="status">Ready</p>
</div>

<div class="card">
  <h3>Custom height correction</h3>

  <label for="offsetInput">Offset in millimetres (mm)</label>
  <input id="offsetInput" type="text" inputmode="numeric"
    autocomplete="off" spellcheck="false" value="0"
    placeholder="Enter correction in whole millimetres">

  <button onclick="saveOffset()">Save offset</button>
  <button class="secondary" onclick="resetOffset()">Reset to zero</button>

  <p><small>
    Enter a positive or negative correction directly.
    Enter a whole number of millimetres (for example, 12 or -5).
    The offset is stored in millimetres regardless of the display unit.
  </small></p>
</div>

<div class="card">
  <h3>Measurement history</h3>
  <a href="/csv">Export CSV</a>
  <button class="secondary" onclick="clearHistory()">Clear history</button>

  <div class="table-wrap">
    <table>
      <thead>
        <tr>
          <th>#</th>
          <th>Head distance</th>
          <th>Height</th>
        </tr>
      </thead>
      <tbody id="history"></tbody>
    </table>
  </div>
</div>

<script>
let busy = false;

function convert(mm) {
  const unit = document.getElementById('unit').value;
  if (unit === 'mm') return mm;
  if (unit === 'cm') return mm / 10;
  if (unit === 'in') return mm / 25.4;
  return mm / 304.8; // Decimal feet, used only for calculations/CSV.
}

function unitName() {
  return document.getElementById('unit').value;
}

function formatValue(mm) {
  const unit = unitName();

  if (unit === 'ft') {
    // Human height notation: 5 ft 11.0 in, NOT decimal feet (5.99 ft).
    // Convert total millimetres to inches, then split inches into base-12 feet.
    let totalInches = mm / 25.4;
    let wholeFeet = Math.floor(totalInches / 12);
    let remainingInches = totalInches - (wholeFeet * 12);
    remainingInches = Math.round(remainingInches * 10) / 10;

    // Carry a rounded 12.0 inches into the next foot.
    if (remainingInches >= 12.0) {
      wholeFeet++;
      remainingInches = 0;
    }
    if (remainingInches < 0) {
      wholeFeet--;
      remainingInches += 12;
    }
    return wholeFeet + ' ft ' + remainingInches.toFixed(1) + ' in';
  }

  return convert(mm).toFixed(2) + ' ' + unit;
}

async function runAction(path) {
  if (busy) return;

  busy = true;
  document.getElementById('measureBtn').disabled = true;
  document.getElementById('calibrateBtn').disabled = true;
  document.getElementById('message').textContent = 'Processing...';

  try {
    const response = await fetch(path, { method: 'POST' });
    const data = await response.json();

    document.getElementById('message').textContent = data.message;
    await refresh();
  } catch (e) {
    document.getElementById('message').textContent =
      'Request failed. Refresh the page and try again.';
  }

  busy = false;
  document.getElementById('measureBtn').disabled = false;
  document.getElementById('calibrateBtn').disabled = false;
}

async function saveOffset() {
  const field = document.getElementById('offsetInput');
  const raw = field.value.trim();
  // Accept signed whole millimetres only; avoid silently rounding typed values.
  if (!/^[+-]?\d+$/.test(raw)) {
    document.getElementById('message').textContent =
      'Enter a whole-number offset in millimetres, for example 12 or -5.';
    return;
  }
  const value = Number(raw);
  if (!Number.isSafeInteger(value) || value < -10000 || value > 10000) {
    document.getElementById('message').textContent =
      'Enter an offset from -10000 to +10000 mm.';
    return;
  }

  try {
    const response = await fetch('/offset', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: 'value=' + encodeURIComponent(value)
    });

    const data = await response.json();
    document.getElementById('message').textContent = data.message;
    if (response.ok) {
      field.value = String(value);
      field.blur();
    }
    await refresh();
  } catch (e) {
    document.getElementById('message').textContent =
      'Could not save the offset.';
  }
}

async function resetOffset() {
  document.getElementById('offsetInput').value = '0';

  try {
    const response = await fetch('/offset', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: 'value=0'
    });

    const data = await response.json();
    document.getElementById('message').textContent = data.message;
    if (response.ok) {
      document.getElementById('offsetInput').value = '0';
      document.getElementById('offsetInput').blur();
    }
    await refresh();
  } catch (e) {
    document.getElementById('message').textContent =
      'Could not reset the offset.';
  }
}

async function clearHistory() {
  if (!confirm('Clear measurement history?')) return;

  try {
    const response = await fetch('/clear-history', { method: 'POST' });
    const data = await response.json();

    document.getElementById('message').textContent = data.message;
    await refresh();
  } catch (e) {
    document.getElementById('message').textContent =
      'Could not clear history.';
  }
}

async function refresh() {
  try {
    const response = await fetch('/data');
    const d = await response.json();

    document.getElementById('height').textContent =
      d.height < 0 ? '--' : formatValue(d.height);

    document.getElementById('distance').textContent =
      d.distance > 0 ? formatValue(d.distance) : '--';

    document.getElementById('floor').textContent =
      d.calibrated ? formatValue(d.floor) : 'Not calibrated';

    document.getElementById('offsetDisplay').textContent =
      d.offset + ' mm';

    // The page polls every 2 seconds. Never overwrite an offset while
    // the user is editing it; this was causing typed digits to disappear.
    const offsetField = document.getElementById('offsetInput');
    if (document.activeElement !== offsetField) {
      offsetField.value = String(d.offset);
    }

    document.getElementById('sensor').textContent =
      d.sensorOK ? 'Ready' : 'Error';

    document.getElementById('calibration').textContent =
      d.calibrated ? 'Calibrated' : 'Required';

    document.getElementById('action').textContent = d.action;

    document.getElementById('history').innerHTML =
      d.history.map((x, i) =>
        '<tr><td>' + (i + 1) + '</td><td>' +
        formatValue(x.distance) + '</td><td>' +
        formatValue(x.height) + '</td></tr>'
      ).join('');
  } catch (e) {
    document.getElementById('sensor').textContent =
      'Device unreachable';
  }
}

refresh();
setInterval(refresh, 2000);
</script>
</body>
</html>
)HTML";

// =====================================================
// WEB API
// =====================================================

void sendActionResult(bool success, const String &message) {
  String json = "{\"success\":";
  json += success ? "true" : "false";
  json += ",\"message\":\"";
  json += message;
  json += "\"}";

  server.send(success ? 200 : 400,
              "application/json",
              json);
}

void handleRoot() {
  server.send_P(200, "text/html", PAGE);
}

void handleData() {
  String json = "{";

  json += "\"floor\":" + String(floorReference);
  json += ",\"distance\":" + String(lastHeadDistance);
  json += ",\"height\":" + String(lastHeightMM, 1);
  json += ",\"offset\":" + String(offsetMM);

  json += ",\"calibrated\":";
  json += calibrated ? "true" : "false";

  json += ",\"sensorOK\":";
  json += sensorOK ? "true" : "false";

  json += ",\"action\":\"";
  json += lastAction;
  json += "\"";

  json += ",\"history\":[";

  for (int i = 0; i < historyCount; i++) {
    int idx = (historyNext - historyCount + i + HISTORY_SIZE)
              % HISTORY_SIZE;

    if (i > 0) json += ",";

    json += "{\"distance\":";
    json += String(history[idx].distanceMM);
    json += ",\"height\":";
    json += String(history[idx].heightMM, 1);
    json += "}";
  }

  json += "]}";

  server.send(200, "application/json", json);
}

void handleWebCalibrate() {
  bool ok = calibrateFloor();

  sendActionResult(
    ok,
    ok ? "Floor calibrated. Beep confirms success."
       : "Calibration failed. Check sensor and target."
  );
}

void handleWebMeasure() {
  bool ok = measureHeight();

  sendActionResult(
    ok,
    ok ? "Measurement completed."
       : lastAction
  );
}

void handleOffset() {
  if (!server.hasArg("value")) {
    sendActionResult(false, "Missing offset value.");
    return;
  }

  String input = server.arg("value");
  char *endPtr = nullptr;

  double parsed = strtod(input.c_str(), &endPtr);

  if (input.length() == 0 ||
      endPtr == input.c_str() ||
      *endPtr != '\0' ||
      !isfinite(parsed) ||
      parsed < -10000.0 ||
      parsed > 10000.0) {
    sendActionResult(false,
                     "Offset must be between -10000 and +10000 mm.");
    return;
  }

  // Stored as whole millimetres.
  offsetMM = (int)lround(parsed);
  prefs.putInt("offset", offsetMM);

  if (calibrated && lastHeadDistance > 0) {
    lastHeightMM = (float)floorReference
                 - (float)lastHeadDistance
                 + (float)offsetMM;

    if (lastHeightMM < 0) lastHeightMM = 0;
  }

  lastAction = "Offset saved";

  sendActionResult(true, "Offset saved: " +
                    String(offsetMM) + " mm.");
}

void handleCSV() {
  String csv =
    "Number,Time_ms,Head_distance_mm,Height_mm,Height_cm,Height_inches,Height_feet\n";

  for (int i = 0; i < historyCount; i++) {
    int idx = (historyNext - historyCount + i + HISTORY_SIZE)
              % HISTORY_SIZE;

    float h = history[idx].heightMM;

    csv += String(i + 1) + ",";
    csv += String(history[idx].timeMs) + ",";
    csv += String(history[idx].distanceMM) + ",";
    csv += String(h, 1) + ",";
    csv += String(h / 10.0f, 2) + ",";
    csv += String(h / 25.4f, 2) + ",";
    csv += String(h / 304.8f, 3) + "\n";
  }

  server.sendHeader(
    "Content-Disposition",
    "attachment; filename=height_history.csv"
  );

  server.send(200, "text/csv", csv);
}

void handleClearHistory() {
  historyCount = 0;
  historyNext = 0;
  lastHeadDistance = 0;
  lastHeightMM = -1.0f;

  lastAction = "History cleared";

  sendActionResult(true, "Measurement history cleared.");
}

void startAccessPoint() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  IPAddress localIP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.softAPConfig(localIP, gateway, subnet);

  apStarted = WiFi.softAP(
    AP_SSID,
    AP_PASSWORD,
    1,      // 2.4 GHz channel 1
    false,  // SSID broadcast enabled
    4       // Maximum clients
  );

  if (apStarted) {
    Serial.println("Access point started.");
    Serial.print("Wi-Fi name: ");
    Serial.println(AP_SSID);
    Serial.print("Password: ");
    Serial.println(AP_PASSWORD);
    Serial.print("Website: http://");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("ERROR: Access point failed to start.");
  }
}



void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("ESP32-WROOM Digital Height Meter");
  Serial.println("Starting system...");
  Serial.println("Board: standard ESP32-WROOM / ESP32 DevKit");
  Serial.println("Pins: SDA=13, SCL=14, BUTTON=27, LED=26, BUZZER=25");

  // Configure outputs first and force them LOW immediately.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("[1] GPIO configured");

  Serial.println("[2] Starting Wi-Fi access point...");
  startAccessPoint();
  Serial.println("[3] Access-point startup function returned");

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/calibrate", HTTP_POST, handleWebCalibrate);
  server.on("/measure", HTTP_POST, handleWebMeasure);
  server.on("/offset", HTTP_POST, handleOffset);
  server.on("/csv", HTTP_GET, handleCSV);
  server.on("/clear-history", HTTP_POST, handleClearHistory);
  server.begin();
  Serial.println("[4] Website server ready");
  Serial.println("Open http://192.168.4.1 when connected to the device Wi-Fi");

  Serial.println("[5] Starting I2C on SDA=13, SCL=14...");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Serial.println("[6] I2C initialized");

  Serial.println("[7] Opening Preferences...");
  if (prefs.begin("heightmeter", false)) {
    floorReference = prefs.getUShort("floor", 0);
    calibrated = prefs.getBool("calibrated", false);
    offsetMM = prefs.getInt("offset", 0);
    if (floorReference < MIN_DISTANCE_MM || floorReference > MAX_DISTANCE_MM) {
      calibrated = false;
    }
    Serial.println("[8] Preferences ready");
  } else {
    calibrated = false;
    floorReference = 0;
    offsetMM = 0;
    Serial.println("WARNING: Preferences unavailable; calibration will not persist.");
  }

  Serial.println("[9] Initializing VL53L1X...");
  sensor.setTimeout(180);
  if (sensor.init()) {
    sensorOK = sensor.setDistanceMode(VL53L1X::Long) &&
               sensor.setMeasurementTimingBudget(50000);
    if (sensorOK) {
      sensor.startContinuous(60);
      sensorOK = (sensor.last_status == 0);
    }
    Serial.println(sensorOK ? "VL53L1X ready." :
                               "ERROR: VL53L1X ranging configuration failed.");
  } else {
    sensorOK = false;
    Serial.println("ERROR: VL53L1X not detected. Check 3V3, GND, SDA=13, SCL=14.");
  }

  Serial.println("[10] Startup completed");
  Serial.println("Measurement filter: up to 2 attempts; each uses 2 independent windows x 21 valid samples.");
  Serial.println("Each window discards 5 initial frames, applies median/MAD filtering, and allows SD up to 5.5 mm; window agreement limit is 8 mm.");
  Serial.println("Wi-Fi: HeightMeter-Setup | Password: height123");
  Serial.println("Hold button for 2 seconds: calibrate immediately.");
  Serial.println("Short press and release: measure height.");
}


void loop() {
  server.handleClient();
  handleButton();
  delay(5);
}
