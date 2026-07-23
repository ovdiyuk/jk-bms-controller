#include <Arduino.h>

// ===== JK BMS UART control (old "NW" TTL protocol, GPS port, 115200 8N1) =====
// Button on D2 -> GND, internal pull-up.
// Pressed  (LOW)  -> discharge MOSFET ON
// Released (HIGH) -> discharge MOSFET OFF
//
// Wiring (recommended):
// - Connect BMS GPS TX -> MCU RX (D0)
// - Connect BMS GPS RX -> MCU TX (D1)
// - Common GND between BMS and MCU
// Note: on ATmega328 (Pro Mini) D0/D1 are the hardware UART used by `Serial`.
// If you use the USB-serial adapter for programming, disconnect BMS TX/RX
// during upload or use a separate adapter.

const uint8_t PIN_BUTTON = 2;
const uint8_t LED_PIN = 13; // onboard LED on Pro Mini (D13)

const uint8_t REG_CHARGE_SW    = 0xAB; // 0/1 - charging MOSFET (reserved)
const uint8_t REG_DISCHARGE_SW = 0xAC; // 0/1 - discharging MOSFET

const unsigned long DEBOUNCE_MS = 50;
const unsigned long REFRESH_MS  = 5000; // periodic resend of state

bool lastStablePressed = false;
bool lastRawPressed    = false;
unsigned long lastEdgeMs    = 0;
unsigned long lastRefreshMs = 0;

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

  // Use BMS serial wrapper to allow alternative UARTs
#if defined(UBRR1H)
  Serial1.write(f, i);
  Serial1.flush();
#else
  Serial.write(f, i);
  Serial.flush();
#endif
}

void setDischarge(bool on) {
  jkWriteRegister(REG_DISCHARGE_SW, on ? 0x01 : 0x00);
  // local LED indicator for debugging (lights when discharge enabled)
  digitalWrite(LED_PIN, on ? HIGH : LOW);
#if defined(UBRR1H)
  // If Serial1 exists, print diagnostic to USB-serial via Serial
  Serial.print("Discharge set: "); Serial.println(on ? "ON" : "OFF");
#else
  Serial.print("Discharge set: "); Serial.println(on ? "ON" : "OFF");
#endif
}

void setup() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // Initialize the BMS UART. If the MCU has Serial1 (separate HW UART), use it
  // to avoid conflict with USB-Serial on `Serial`.
#if defined(UBRR1H)
  Serial1.begin(115200);
#else
  Serial.begin(115200);
#endif
  delay(300); // даємо BMS/лінії заспокоїтись після ресету

  lastRawPressed = lastStablePressed = (digitalRead(PIN_BUTTON) == LOW);
  setDischarge(lastStablePressed);
  lastRefreshMs = millis(); // запобігаємо негайному повтору в loop()
}

void loop() {
  // BMS може відповідати на кожну команду - зливаємо вхідний буфер
#if defined(UBRR1H)
  while (Serial1.available()) Serial1.read();
#else
  while (Serial.available()) Serial.read();
#endif

  bool raw = (digitalRead(PIN_BUTTON) == LOW);
  unsigned long now = millis();

  if (raw != lastRawPressed) {
    lastRawPressed = raw;
    lastEdgeMs = now;
  }

  // стан тримається довше дебаунсу і відрізняється від зафіксованого
  if ((now - lastEdgeMs) >= DEBOUNCE_MS && raw != lastStablePressed) {
    lastStablePressed = raw;
    setDischarge(lastStablePressed);
    lastRefreshMs = now;
  }

  // страхувальний повтор поточного стану
  if ((now - lastRefreshMs) >= REFRESH_MS) {
    setDischarge(lastStablePressed);
    lastRefreshMs = now;
  }
}
