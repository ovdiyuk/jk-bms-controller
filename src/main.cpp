#include <Arduino.h>

// ===== JK BMS UART control (old "NW" TTL protocol, GPS port, 115200 8N1) =====
// RED/YELLOW/GREEN indicator, common cathode, built-in resistors (3-6 V module).
// Latching switch on GPIO1 -> 3.3 V (INPUT_PULLDOWN)
// HIGH (3.3 V) -> discharge ON
// LOW  (open)  -> discharge OFF
//
// Wiring (as actually built):
// - GPIO3 (MCU RX) <- TX of the optical module   [module TX1]
// - GPIO4 (MCU TX) -> RX of the optical module   [module RX1]
// - GPIO5 -> RED LED
// - GPIO6 -> YELLOW LED
// - GPIO7 -> GREEN LED
// - GPIO1 -> latching switch to 3.3 V
// - GPIO2 -> SPARE, strapping pin, leave unconnected
//
// Indicator states (exactly one colour at a time):
//   YELLOW solid - ready:   no faults, discharge OFF
//   GREEN  solid - running: no faults, discharge ON, SOC > 20%
//   GREEN  blink - running: no faults, discharge ON, SOC <= 20%
//   RED    solid - running with faults: faults present, discharge ON
//   RED    blink - stopped with faults: faults present, discharge OFF
//   all three blink at 300 ms - no link to the BMS
// - Common GND between the MCU and its optical module only.
//   The BMS side is galvanically isolated: the link is a single WDM fiber.

const uint8_t PIN_BUTTON = 1;         // GPIO1 - latching switch to 3.3 V
const uint8_t BMS_RX_PIN = 3;         // GPIO3 (BMS RX)
const uint8_t BMS_TX_PIN = 4;         // GPIO4 (BMS TX)
const bool BUTTON_ACTIVE_LEVEL = HIGH; // HIGH (3.3 V) = discharge ON

const uint8_t LED_PIN_RED = 5;    // GPIO5 - faults
const uint8_t LED_PIN_YELLOW = 6; // GPIO6 - ready
const uint8_t LED_PIN_GREEN = 7;  // GPIO7 - running
const uint8_t PIN_SPARE = 2;          // GPIO2 - SPARE (unused)
const uint8_t ONBOARD_LED_PIN =
    8; // Onboard RED LED - ESP32-C3 Super Mini (Heartbeat)

const uint8_t REG_CHARGE_SW = 0xAB;    // 0/1 - charging MOSFET (reserved)
const uint8_t REG_DISCHARGE_SW = 0xAC; // 0/1 - discharging MOSFET

const unsigned long DEBOUNCE_MS = 50;
const unsigned long REFRESH_MS = 5000; // periodic resend of state
const unsigned long HEARTBEAT_INTERVAL_MS = 500;

// ===== Налаштування порогів індикації (налаштовувані константи) =====
const uint8_t SOC_LOW_THRESHOLD_PERCENT =
    20; // ЗЕЛЕНИЙ: понад цей заряд (%) горить рівно, на ньому й нижче мигає
const unsigned long LED_BLINK_INTERVAL_MS =
    500; // Інтервал мигання зеленого і червоного (мс)

const unsigned long BMS_TIMEOUT_MS = 5000; // Таймаут зв'язку з BMS (мс)
const unsigned long NO_CONN_BLINK_MS = 300; // Інтервал мигання при відсутності зв'язку (мс)

bool targetDischargeState = false;    // Цільовий стан розряду, заданий тумблером
bool bmsActualDischargeState = false; // Реальний стан ключа розряду з BMS (тег 0xAC)
unsigned long lastWriteRetryMs = 0;   // Час останнього повтору команди

float bmsCurrentA = 0.0f;
float bmsTempC = 0.0f;
float bmsMinCellV = 3.300f;
float bmsMaxCellV = 3.300f;
float bmsCellDeltaV = 0.000f;
uint8_t bmsSocPercent = 100;

unsigned long lastBmsRxMs = 0;     // Час останньої відповіді від BMS
bool bmsConnected = false;          // Прапорець наявності зв'язку з BMS
const char *ledState = "?";        // Поточний стан індикатора, для логу

bool lastStablePressed = false;
bool lastRawPressed = false;
unsigned long lastEdgeMs = 0;
unsigned long lastRefreshMs = 0;

void setOnboardLed(bool on) {
  digitalWrite(ONBOARD_LED_PIN,
               on ? LOW
                  : HIGH); // onboard LED is active-low on many ESP32-C3 boards
}

void setupLeds() {
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  setOnboardLed(false);

  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_YELLOW, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);

  digitalWrite(LED_PIN_RED, LOW);
  digitalWrite(LED_PIN_YELLOW, LOW);
  digitalWrite(LED_PIN_GREEN, LOW);
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

// Мигання всіх LED при відсутності зв'язку з BMS
void updateNoConnectionBlink() {
  static unsigned long lastBlinkMs = 0;
  static bool blinkState = false;

  if (millis() - lastBlinkMs >= NO_CONN_BLINK_MS) {
    lastBlinkMs = millis();
    blinkState = !blinkState;
  }

  digitalWrite(LED_PIN_RED, blinkState ? HIGH : LOW);
  digitalWrite(LED_PIN_YELLOW, blinkState ? HIGH : LOW);
  digitalWrite(LED_PIN_GREEN, blinkState ? HIGH : LOW);
}

// Визначення активного коду помилки (1..6) за реальними фізичними параметрами
uint8_t determineActiveFaultCode() {
  // 1: Перенапруга / Перерозряд осередку (<2.60V або >3.65V)
  if (bmsMinCellV > 0.5f && (bmsMinCellV < 2.60f || bmsMaxCellV > 3.65f)) return 1;

  // 2: Перевищення струму розряду / заряду (>120A)
  if (fabs(bmsCurrentA) > 120.0f) return 2;

  // 3: Перегрів або замерзання батареї (>65°C або <0°C)
  if (bmsTempC > 65.0f || bmsTempC < 0.0f) return 3;

  // 4: Перегрів транзисторів MOSFET (>85°C)
  if (bmsTempC > 85.0f) return 4;

  // 5: Аварія короткого замикання (струм > 250A)
  if (fabs(bmsCurrentA) > 250.0f) return 5;

  // 6: Аварійне розбалансування (різниця осередків Delta V > 0.150V / 150 мВ)
  if (bmsCellDeltaV > 0.150f) return 6;

  return 0; // Немає аварій
}

// Неблокуюча індикація серіями коротких моргань на Жовтому LED (GPIO7)
void updateLedIndicators() {
  // Перевірка зв'язку з BMS
  bmsConnected = (lastBmsRxMs > 0) && (millis() - lastBmsRxMs < BMS_TIMEOUT_MS);

  // Немає зв'язку — мигаємо всіма трьома, це має пріоритет над усім іншим
  if (!bmsConnected) {
    updateNoConnectionBlink();
    return;
  }

  static unsigned long lastBlinkMs = 0;
  static bool blinkState = false;
  if (millis() - lastBlinkMs >= LED_BLINK_INTERVAL_MS) {
    lastBlinkMs = millis();
    blinkState = !blinkState;
  }

  const uint8_t faultCode = determineActiveFaultCode();
  // Беремо РЕАЛЬНИЙ стан ключа з BMS, а не положення тумблера:
  // індикатор має показувати те, що є, а не те, що ми наказали.
  const bool discharging = bmsActualDischargeState;

  // Лог коду аварії при зміні — сам код на індикаторі більше не показується
  static uint8_t lastLoggedFault = 0xFF;
  if (faultCode != lastLoggedFault) {
    lastLoggedFault = faultCode;
    if (faultCode == 0) {
      Serial.println("FAULT cleared");
    } else {
      Serial.printf("FAULT code %u active\n", faultCode);
    }
  }

  bool red = false, yellow = false, green = false;

  if (faultCode != 0) {
    // Є аварія: працює з помилками — рівно, не працює з помилками — мигає
    red = discharging ? true : blinkState;
    ledState = discharging ? "RED solid  (faults, running)"
                           : "RED blink  (faults, stopped)";
  } else if (discharging) {
    // Аварій немає, розряд активний: понад поріг — рівно, на порозі й нижче — мигає
    green = (bmsSocPercent > SOC_LOW_THRESHOLD_PERCENT) ? true : blinkState;
    ledState = (bmsSocPercent > SOC_LOW_THRESHOLD_PERCENT)
                   ? "GREEN solid (running)"
                   : "GREEN blink (running, SOC low)";
  } else {
    // Аварій немає, розряд вимкнений: батарея готова
    yellow = true;
    ledState = "YELLOW solid (ready)";
  }

  digitalWrite(LED_PIN_RED, red ? HIGH : LOW);
  digitalWrite(LED_PIN_YELLOW, yellow ? HIGH : LOW);
  digitalWrite(LED_PIN_GREEN, green ? HIGH : LOW);
}

// Формує та шле фрейм запису одного регістра:
// 4E 57 | len(2) | terminal(4) | cmd=02 | src=03 | type=00 | reg | val |
// record(4) | 68 | crc(4)
void jkWriteRegister(uint8_t reg, uint8_t value) {
  uint8_t f[26];
  uint8_t i = 0;
  f[i++] = 0x4E;
  f[i++] = 0x57; // header "NW"
  f[i++] = 0x00;
  f[i++] = 0x14; // length = 20 (все після header)
  f[i++] = 0x00;
  f[i++] = 0x00;
  f[i++] = 0x00;
  f[i++] = 0x00; // terminal no
  f[i++] = 0x02; // command: write register
  f[i++] = 0x03; // frame source: host/PC
  f[i++] = 0x00; // transmission type
  f[i++] = reg;
  f[i++] = value;
  f[i++] = 0x00;
  f[i++] = 0x00;
  f[i++] = 0x00;
  f[i++] = 0x00; // record no
  f[i++] = 0x68; // end flag

  uint16_t sum = 0;
  for (uint8_t k = 0; k < i; k++)
    sum += f[k];
  // CRC: two high bytes 0x00 0x00 then 16-bit sum (hi, lo)
  f[i++] = 0x00;
  f[i++] = 0x00;
  f[i++] = (uint8_t)(sum >> 8);
  f[i++] = (uint8_t)(sum & 0xFF);

  Serial.print("TX JK: ");
  for (uint8_t k = 0; k < i; ++k) {
    if (f[k] < 0x10)
      Serial.print('0');
    Serial.print(f[k], HEX);
    Serial.print(' ');
  }
  Serial.println();

  Serial1.write(f, i);
  Serial1.flush();
}

void setDischarge(bool on) {
  targetDischargeState = on;
  jkWriteRegister(REG_DISCHARGE_SW, on ? 0x01 : 0x00);

  Serial.printf(">>> SWITCH TOGGLED! Target Discharge = %s <<<\n",
                on ? "ON" : "OFF");
}

void refreshDischargeState() {
  jkWriteRegister(REG_DISCHARGE_SW, targetDischargeState ? 0x01 : 0x00);
}

// Надсилання запиту телеметрії (струм, напруга, SOC, температура) до JK BMS
void jkRequestTelemetry() {
  static const uint8_t reqFrame[21] = {
      0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x06, 0x03, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x29};
  Serial1.write(reqFrame, sizeof(reqFrame));
  Serial1.flush();
}

void parseTelemetryData(const uint8_t *buf, uint16_t len) {
  uint16_t k = 11; // Корисне навантаження кадру телеметрії починається з 11 байта
  float maxTemp = -99.0f;
  uint16_t newAlarmWord = 0;

  while (k < len - 4) {
    uint8_t tag = buf[k];

    // Тег 0x79: Напруги осередків (Cell Voltages) - перший байт довжина
    if (tag == 0x79 && k + 1 < len) {
      uint8_t cellLen = buf[k + 1];
      uint8_t cellCount = cellLen / 3;
      float minV = 99.0f;
      float maxV = 0.0f;

      for (uint8_t c = 0; c < cellCount; c++) {
        uint16_t idx = k + 2 + c * 3;
        if (idx + 2 < len) {
          uint16_t cellMv = ((uint16_t)buf[idx + 1] << 8) | buf[idx + 2];
          float cellV = (float)cellMv * 0.001f;
          if (cellV > 0.5f && cellV < minV) minV = cellV;
          if (cellV > maxV) maxV = cellV;
        }
      }
      if (minV < 90.0f) bmsMinCellV = minV;
      if (maxV > 0.0f) bmsMaxCellV = maxV;
      bmsCellDeltaV = bmsMaxCellV - bmsMinCellV;

      k += 2 + cellLen; // Пропускаємо тег, байт довжини та самі напруги
    }
    // Тег 0x80, 0x81, 0x82: Температура (°C) - 2 байти
    else if ((tag == 0x80 || tag == 0x81 || tag == 0x82) && k + 2 < len - 4) {
      int16_t rawTemp = (int16_t)(((uint16_t)buf[k + 1] << 8) | buf[k + 2]);
      if (rawTemp > 100)
        rawTemp -= 100;
      if (rawTemp >= -40 && rawTemp <= 120) {
        if ((float)rawTemp > maxTemp) {
          maxTemp = (float)rawTemp;
        }
      }
      k += 3;
    }
    // Тег 0x83: Загальна напруга - 2 байти
    else if (tag == 0x83 && k + 2 < len - 4) {
      k += 3;
    }
    // Тег 0x84: Струм (Current) - 2 байти
    else if (tag == 0x84 && k + 2 < len - 4) {
      uint16_t rawVal = ((uint16_t)buf[k + 1] << 8) | buf[k + 2];
      if (rawVal & 0x8000) {
        bmsCurrentA = (float)(rawVal & 0x7FFF) * 0.01f;
      } else {
        bmsCurrentA = -(float)(rawVal & 0x7FFF) * 0.01f;
      }
      k += 3;
    }
    // Тег 0x85: Заряд батареї (SOC %) - 1 байт
    else if (tag == 0x85 && k + 1 < len - 4) {
      bmsSocPercent = buf[k + 1];
      k += 2;
    }
    // Тег 0x86: Кількість датчиків температури - 1 байт
    else if (tag == 0x86 && k + 1 < len - 4) {
      k += 2;
    }
    // Тег 0x87: Кількість циклів - 2 байти
    else if (tag == 0x87 && k + 2 < len - 4) {
      k += 3;
    }
    // Тег 0x89: Накопичена ємність / енергія - 4 байти
    else if (tag == 0x89 && k + 4 < len - 4) {
      k += 5;
    }
    // Тег 0x8A: Кількість осередків - 1 байт
    else if (tag == 0x8A && k + 1 < len - 4) {
      k += 2;
    }
    // Тег 0x8B, 0x8C, 0x8D: Мін/Макс напруга осередка та Delta V - 2 байти
    else if ((tag == 0x8B || tag == 0x8C || tag == 0x8D) && k + 2 < len - 4) {
      k += 3;
    }
    // Тег 0xAB, 0xAC: Стан ключів заряду та розряду (1 байт)
    else if (tag == 0xAB && k + 1 < len - 4) {
      k += 2;
    }
    else if (tag == 0xAC && k + 1 < len - 4) {
      bmsActualDischargeState = (buf[k + 1] != 0x00);
      k += 2;
    }
    // Тег 0x8E: Аварії / Захисти (Alarm Word 1) - 2 байти
    else if (tag == 0x8E && k + 2 < len - 4) {
      newAlarmWord = ((uint16_t)buf[k + 1] << 8) | buf[k + 2];
      k += 3;
    }
    // Тег 0x8F: Попередження (Alarm Word 2) - 2 байти
    else if (tag == 0x8F && k + 2 < len - 4) {
      k += 3;
    } else {
      k++; // Пропускаємо 1 байт для невідомого тегу
    }
  }

  if (maxTemp > -90.0f) {
    bmsTempC = maxTemp;
  }
}

void processBmsResponses() {
  static uint8_t rxBuf[320];
  static uint16_t rxIdx = 0;
  static uint16_t expectedLen = 0;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    // Синхронізація кадру за заголовком 0x4E 0x57
    if (rxIdx == 0 && b != 0x4E)
      continue;
    if (rxIdx == 1 && b != 0x57) {
      rxIdx = 0;
      continue;
    }

    rxBuf[rxIdx++] = b;

    if (rxIdx == 4) {
      expectedLen = ((uint16_t)rxBuf[2] << 8) | rxBuf[3];
      expectedLen += 4; // Header (2) + Len bytes (2)
    }

    // Якщо це короткий кадр підтвердження запису (21 байт)
    if (rxIdx == 21 && rxBuf[8] == 0x02) {
      uint8_t cmd = rxBuf[8];
      uint8_t status = rxBuf[9];
      uint8_t reg = rxBuf[11];

      if (cmd == 0x02 && status == 0x00 && reg == REG_DISCHARGE_SW) {
        bmsActualDischargeState = targetDischargeState;
        lastBmsRxMs = millis();

        Serial.printf("BMS Confirmed Discharge Write = %s\n",
                      bmsActualDischargeState ? "ON" : "OFF");
      }
      rxIdx = 0;
      expectedLen = 0;
    }
    // Якщо це довгий кадр телеметрії (0x06 / 0x03)
    else if (expectedLen > 21 && rxIdx >= expectedLen) {
      if (rxBuf[8] == 0x06 || rxBuf[8] == 0x03) {
        Serial.print("RX TEL [");
        Serial.print(rxIdx);
        Serial.print("]: ");
        for (uint16_t k = 0; k < rxIdx; ++k) {
          if (rxBuf[k] < 0x10)
            Serial.print('0');
          Serial.print(rxBuf[k], HEX);
          Serial.print(' ');
        }
        Serial.println();

        parseTelemetryData(rxBuf, rxIdx);
        lastBmsRxMs = millis(); // Оновлюємо час останньої відповіді
        Serial.printf(
            "BMS Status: Current = %.2f A | SOC = %d%% | Temp = %.1f C | MinV = %.3f V | MaxV = %.3f V | DeltaV = %.3f V | DSG = %s | LED: %s\n",
            bmsCurrentA, bmsSocPercent, bmsTempC, bmsMinCellV, bmsMaxCellV, bmsCellDeltaV,
            bmsActualDischargeState ? "ON " : "OFF", ledState);
      }
      rxIdx = 0;
      expectedLen = 0;
    }

    if (rxIdx >= sizeof(rxBuf)) {
      rxIdx = 0;
      expectedLen = 0;
    }
  }
}

void setup() {
  pinMode(PIN_BUTTON, INPUT_PULLDOWN); // switch pulls the pin UP to 3.3 V
  setupLeds();

  Serial.begin(115200);
  Serial.println("JK BMS controller starting");
  Serial1.begin(115200, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
  Serial.printf("Serial1 initialized: RX=GPIO%d, TX=GPIO%d\n", BMS_RX_PIN,
                BMS_TX_PIN);
  delay(300);

  lastRawPressed = lastStablePressed =
      (digitalRead(PIN_BUTTON) == BUTTON_ACTIVE_LEVEL);
  setDischarge(lastStablePressed);
  lastRefreshMs = millis();
}

void loop() {
  processBmsResponses();

  updateHeartbeat();
  updateLedIndicators();

  bool raw = (digitalRead(PIN_BUTTON) == BUTTON_ACTIVE_LEVEL);
  unsigned long now = millis();

  if (raw != lastRawPressed) {
    lastRawPressed = raw;
    lastEdgeMs = now;
  }

  if ((now - lastEdgeMs) >= DEBOUNCE_MS && raw != targetDischargeState) {
    targetDischargeState = raw;
    Serial.printf(">>> SWITCH TOGGLED! Target Discharge State = %s <<<\n",
                  targetDischargeState ? "ON" : "OFF");
    jkWriteRegister(REG_DISCHARGE_SW, targetDischargeState ? 0x01 : 0x00);
    lastWriteRetryMs = now;
  }

  // АВТОМАТИЧНИЙ ПОВТОР (RETRY):
  // Якщо реальний стан BMS НЕ відповідає положенню тумблера — повертаємо команду кожні 300 мс до підтвердження!
  if (bmsConnected && (bmsActualDischargeState != targetDischargeState)) {
    if (now - lastWriteRetryMs >= 300) {
      lastWriteRetryMs = now;
      jkWriteRegister(REG_DISCHARGE_SW, targetDischargeState ? 0x01 : 0x00);
      Serial.printf("--> Resending Discharge command (%s) to match switch!\n",
                    targetDischargeState ? "ON" : "OFF");
    }
  }

  // Періодичний запит телеметрії від BMS (5 разів на секунду)
  static unsigned long lastTelemetryReqMs = 0;
  if ((now - lastTelemetryReqMs) >= 200) {
    lastTelemetryReqMs = now;
    jkRequestTelemetry();
  }
}
