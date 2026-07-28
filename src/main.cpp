#include <Arduino.h>

// ===== JK BMS UART control (old "NW" TTL protocol, GPS port, 115200 8N1) =====
// Button on GPIO1 -> 3.3V / GND.
// HIGH (3.3V) -> discharge ON
// LOW  (0V)   -> discharge OFF
//
// Wiring:
// - GPIO3 -> BMS RX
// - GPIO4 -> BMS TX
// - GPIO5 -> discharge active LED
// - GPIO6 -> current > 5 A LED
// - GPIO7 -> current < 0 A (charging) LED
// - GPIO10 -> MOS/temp fault LED
// - GPIO20 -> SOC LED (solid >= 30%, blink < 30%)
// - GPIO1 -> button
// - Common GND between BMS and MCU

const uint8_t PIN_BUTTON = 1; // GPIO1 is the original button pin for this board
const uint8_t BMS_RX_PIN = 3;
const uint8_t BMS_TX_PIN = 4;
const bool BUTTON_ACTIVE_LEVEL = HIGH;
const uint8_t LED_PIN_DISCHARGE = 5;
const uint8_t LED_PIN_CURRENT_HIGH = 6;
const uint8_t LED_PIN_CHARGE = 7;
const uint8_t LED_PIN_TEMP = 10;
const uint8_t LED_PIN_SOC = 20;
const uint8_t ONBOARD_LED_PIN = 8; // onboard LED on ESP32-C3 Super Mini

const uint8_t REG_CHARGE_SW    = 0xAB; // 0/1 - charging MOSFET (reserved)
const uint8_t REG_DISCHARGE_SW = 0xAC; // 0/1 - discharging MOSFET

const unsigned long DEBOUNCE_MS = 50;
const unsigned long REFRESH_MS  = 5000; // periodic resend of state
const unsigned long HEARTBEAT_INTERVAL_MS = 500;
const unsigned long SOC_BLINK_INTERVAL_MS = 500;

bool dischargeEnabled = false;
float bmsCurrentA = 0.0f;
float bmsTempC = 0.0f;
uint8_t bmsSocPercent = 100;

bool lastStablePressed = false;
bool lastRawPressed    = false;
unsigned long lastEdgeMs    = 0;
unsigned long lastRefreshMs = 0;

void setOnboardLed(bool on) {
  digitalWrite(ONBOARD_LED_PIN, on ? LOW : HIGH); // onboard LED is active-low on many ESP32-C3 boards
}

void setupLeds() {
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  setOnboardLed(false);

  pinMode(LED_PIN_DISCHARGE, OUTPUT);
  pinMode(LED_PIN_CURRENT_HIGH, OUTPUT);
  pinMode(LED_PIN_CHARGE, OUTPUT);
  pinMode(LED_PIN_TEMP, OUTPUT);
  pinMode(LED_PIN_SOC, OUTPUT);

  digitalWrite(LED_PIN_DISCHARGE, LOW);
  digitalWrite(LED_PIN_CURRENT_HIGH, LOW);
  digitalWrite(LED_PIN_CHARGE, LOW);
  digitalWrite(LED_PIN_TEMP, LOW);
  digitalWrite(LED_PIN_SOC, LOW);
}

void updateHeartbeat() {
  static unsigned long lastHeartbeatMs = 0;
  static bool heartbeatState = false;

  if (millis() - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = millis();
    heartbeatState = !heartbeatState;
    setOnboardLed(heartbeatState);
  }
}

void updateLedIndicators() {
  static unsigned long lastSocBlinkMs = 0;
  static bool socBlinkState = false;

  digitalWrite(LED_PIN_DISCHARGE, dischargeEnabled ? HIGH : LOW);
  digitalWrite(LED_PIN_CURRENT_HIGH, (bmsCurrentA > 5.0f) ? HIGH : LOW);
  digitalWrite(LED_PIN_CHARGE, (bmsCurrentA < 0.0f) ? HIGH : LOW);
  digitalWrite(LED_PIN_TEMP, (bmsTempC > 35.0f) ? HIGH : LOW);

  if (bmsSocPercent >= 30) {
    digitalWrite(LED_PIN_SOC, HIGH);
  } else {
    if (millis() - lastSocBlinkMs >= SOC_BLINK_INTERVAL_MS) {
      lastSocBlinkMs = millis();
      socBlinkState = !socBlinkState;
    }
    digitalWrite(LED_PIN_SOC, socBlinkState ? HIGH : LOW);
  }
}

// Формує та шле фрейм запису одного регістра:
// 4E 57 | len(2) | terminal(4) | cmd=02 | src=03 | type=00 | reg | val | record(4) | 68 | crc(4)
void jkWriteRegister(uint8_t reg, uint8_t value) {
  uint8_t f[26];
  uint8_t i = 0;
  f[i++] = 0x4E; f[i++] = 0x57;             // header "NW"
  f[i++] = 0x00; f[i++] = 0x14;             // length = 20 (все після header)
  f[i++] = 0x00; f[i++] = 0x00; f[i++] = 0x00; f[i++] = 0x00; // terminal no
  f[i++] = 0x02;                            // command: write register
  f[i++] = 0x03;                            // frame source: host/PC
  f[i++] = 0x00;                            // transmission type
  f[i++] = reg;
  f[i++] = value;
  f[i++] = 0x00; f[i++] = 0x00; f[i++] = 0x00; f[i++] = 0x00; // record no
  f[i++] = 0x68;                            // end flag

  uint16_t sum = 0;
  for (uint8_t k = 0; k < i; k++) sum += f[k];
  // CRC: two high bytes 0x00 0x00 then 16-bit sum (hi, lo)
  f[i++] = 0x00; f[i++] = 0x00;
  f[i++] = (uint8_t)(sum >> 8); f[i++] = (uint8_t)(sum & 0xFF);

  Serial.print("TX JK: ");
  for (uint8_t k = 0; k < i; ++k) {
    if (f[k] < 0x10) Serial.print('0');
    Serial.print(f[k], HEX);
    Serial.print(' ');
  }
  Serial.println();

  Serial1.write(f, i);
  Serial1.flush();
}

void setDischarge(bool on) {
  dischargeEnabled = on;
  jkWriteRegister(REG_DISCHARGE_SW, on ? 0x01 : 0x00);
  updateLedIndicators();
  Serial.print("Discharge set: "); Serial.println(on ? "ON" : "OFF");
}

void refreshDischargeState() {
  jkWriteRegister(REG_DISCHARGE_SW, dischargeEnabled ? 0x01 : 0x00);
}

void setup() {
  pinMode(PIN_BUTTON, INPUT_PULLDOWN);
  setupLeds();

  Serial.begin(115200);
  Serial.println("JK BMS controller starting");
  Serial1.begin(115200, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
  delay(300); // даємо BMS/лінії заспокоїтись після ресету

  lastRawPressed = lastStablePressed = (digitalRead(PIN_BUTTON) == BUTTON_ACTIVE_LEVEL);
  setDischarge(lastStablePressed);
  lastRefreshMs = millis(); // запобігаємо негайному повтору в loop()
}

void loop() {
  // BMS може відповідати на кожну команду - зливаємо вхідний буфер
  while (Serial1.available()) {
    Serial1.read();
  }

  updateHeartbeat();
  updateLedIndicators();

  bool raw = (digitalRead(PIN_BUTTON) == BUTTON_ACTIVE_LEVEL);
  unsigned long now = millis();

  if (raw != lastRawPressed) {
    lastRawPressed = raw;
    lastEdgeMs = now;
  }

  if ((now - lastEdgeMs) >= DEBOUNCE_MS && raw != lastStablePressed) {
    lastStablePressed = raw;
    setDischarge(lastStablePressed);
    lastRefreshMs = now;
  }

  if ((now - lastRefreshMs) >= REFRESH_MS) {
    lastRefreshMs = now;
    refreshDischargeState();
  }
}
