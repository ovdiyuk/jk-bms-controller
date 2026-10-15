#include <Arduino.h>
#include <esp_system.h>
#include <U8g2lib.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

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
//   red -> yellow -> green chase at 300 ms - no link to the BMS
// - Common GND between the MCU and its optical module only.
//   The BMS side is galvanically isolated: the link is a single WDM fiber.

const uint8_t PIN_BUTTON = 1;         // GPIO1 - latching switch to 3.3 V
const uint8_t BMS_RX_PIN = 3;         // GPIO3 (BMS RX)
const uint8_t BMS_TX_PIN = 4;         // GPIO4 (BMS TX)
const bool BUTTON_ACTIVE_LEVEL = HIGH; // HIGH (3.3 V) = discharge ON

const uint8_t LED_PIN_RED = 5;    // GPIO5 - faults
const uint8_t LED_PIN_YELLOW = 6; // GPIO6 - ready
const uint8_t LED_PIN_GREEN = 7;  // GPIO7 - running

// OLED 1.3" 128x64, SH1106 (NOT SSD1306 - these 1.3" modules use SH1106),
// I2C on the two pins that are still free.
const uint8_t OLED_SDA_PIN = 10;
const uint8_t OLED_SCL_PIN = 20;
const uint32_t OLED_I2C_HZ = 400000;
const unsigned long OLED_REFRESH_MS = 250;

// Калібрування видимої області: корпус вужчий за скло дисплея.
// 1 — малюємо вкладені рамки замість даних, щоб визначити поля.
#define OLED_CALIBRATION 0

// Поля безпечної області в пікселях (уточнюються за фото калібрування).
// Бортик корпусу зрізає край скла нерівномірно, близько 1-4 px.
// Беремо 6 px з запасом: робоча область 116 x 52.
const uint8_t OLED_MARGIN_X = 6;
const uint8_t OLED_MARGIN_Y = 6;
const unsigned long OLED_RENDER_BUDGET_MS = 120; // довше - вважаємо шину завислою
const uint8_t OLED_MAX_FAILS = 5;

// Власний акумулятор пульта: дільник 10k/10k на GPIO0.
// Li-Po 3.0-4.2 V -> 1.5-2.1 V на вході ADC, у межах діапазону.
const uint8_t PIN_VBAT_SENSE = 0;
const float VBAT_DIVIDER = 2.0f;      // (10k + 10k) / 10k — фізичне співвідношення

// Поправка на допуск резисторів і калібрування ADC. Знята по мультиметру
// одночасним виміром: на піні 2084 мВ, прилад показував 4.1511 В.
//
// Корекція однокрапкова, тобто масштабна. Двокрапкову поки побудувати не
// вийшло: обидві наявні точки лежать близько (4.07 і 4.15 В), і на такому
// короткому плечі кілька мВ похибки дають нахил, помилковий на 20%.
// Для справжньої двокрапкової потрібен вимір на розрядженому, біля 3.4 В.
const float VBAT_CALIBRATION = 0.99595f;
const uint8_t VBAT_SAMPLES = 16;      // усереднення проти шуму від бустерів
const unsigned long VBAT_PERIOD_MS = 1000;
const unsigned long STATUS_OVERLAY_MS = 5000; // ARM/DISARM після перемикання

// ===== Точка доступу =====
// Увага: це передавач 2.4 ГГц на пульті, що стоїть поряд з РЕБ. Діапазон
// може бути придушений власною установкою, а пульт стає помітним в ефірі.
// Вимикається одним нулем.
#define WIFI_AP_ENABLED 1
// Облікові дані лежать в include/secrets.h, який не комітиться:
// репозиторій публічний, а через цю мережу керується живлення РЕБ.
// Зразок — include/secrets.h.example.
#if __has_include("secrets.h")
#include "secrets.h"
#else
#define SECRET_AP_SSID "JK-Pult"
#define SECRET_AP_PASS "change-me-please"
#endif
const char *AP_SSID = SECRET_AP_SSID;
const char *AP_PASS = SECRET_AP_PASS;
const uint8_t AP_CHANNEL = 6;
const uint8_t AP_MAX_CLIENTS = 2;

// Опитування BMS адаптивне: у спокої рідко, щоб не їсти батарею пульта,
// але поки команда не підтверджена — часто, щоб ARM/DISARM не чекав секунду.
// На швидкість самого перемикання це не впливає: команда йде по фронту тумблера.
const unsigned long TELEMETRY_IDLE_MS = 1000;
const unsigned long TELEMETRY_FAST_MS = 200;

// Нижче цього струму прогноз часу не має сенсу
const float RUNTIME_MIN_CURRENT_A = 0.5f;

// Пульт не має шунта, тому його час роботи рахується за ПРИЙНЯТИМ
// споживанням, а не виміряним. 320 mA — оцінка з бюджету живлення:
// оптомодуль ~250, контролер ~50, індикатор ~20. Уточнити за фактом.
const uint16_t PULT_CAPACITY_MAH = 2000;
const uint16_t PULT_TYPICAL_DRAW_MA = 320;

// Визначення заряджання пульта.
// 1 — читаємо вихід CHRG зарядного модуля (TP4056 і подібні): він
//     відкритий колектор, притиснутий до нуля поки йде заряд.
//     Підключити до GPIO21, це єдиний вільний нестрапінговий пін.
// 0 — здогадка за трендом напруги. Працює в основній частині заряду,
//     але наприкінці крива виходить на полицю і тренд зникає.
#define PULT_CHARGE_PIN_ENABLED 0
const uint8_t PIN_PULT_CHARGE = 21;

// Підключення зарядного модуля дає СТРИБОК напруги на клемах: виміряно
// 3.65 -> 3.88 В за одну секунду, і стільки ж назад при відєднанні.
// Природна зміна при розряді — одиниці мВ за ХВИЛИНУ, тож поріг 50 мВ
// за секунду відділяє одне від одного із запасом на два порядки.
const float STEP_THRESHOLD_V = 0.050f;

// Повільний тренд лишається запасним варіантом: він ловить випадок,
// коли пульт увімкнули з уже підключеною зарядкою і стрибка не було.
// Швидкість зміни заряду. Вікно росте до 5 хв, потім опорна точка
// зсувається. Коротше вікно потонуло б у шумі: 2 мВ шуму на крутому
// місці кривої — це вже 0.3 % заряду.
// Ковзне вікно: тримаємо 10 зразків по одному на 30 с, тобто 5 хвилин.
// Раніше опорна точка скидалася раз на вікно, і в цей момент швидкість
// стрибала — саме це й виглядало як девіація.
const uint8_t RATE_SLOTS = 10;
const unsigned long RATE_SAMPLE_MS = 30000;
// Відсоток — ціле число, тож один крок квантування на короткому вікні
// дає фантомну швидкість: 1 % за 60 с це вже 60 %/год з нічого.
// Тому чекаємо, поки вікно заповниться повністю: на 5 хвилинах той самий
// крок дає 12 %/год, що вже тоне в усередненні.
const float RATE_MIN_PCT_PER_H = 5.0f;        // нижче — вважаємо спокоєм

// Поверх вікна — ковзне середнє швидкості по 5 оцінках. Усереднюємо саме
// швидкість, а не час: час це залишок/швидкість, і біля нуля він злітає
// в сотні годин, чим зіпсував би середнє.
const uint8_t RATE_AVG_SLOTS = 5;

const unsigned long TREND_WINDOW_MS = 180000;
const float TREND_THRESHOLD_V = 0.005f;
const uint8_t PIN_SPARE = 2;          // GPIO2 - SPARE (unused)
const uint8_t ONBOARD_LED_PIN =
    8; // Onboard RED LED - ESP32-C3 Super Mini (Heartbeat)

const uint8_t REG_CHARGE_SW = 0xAB;    // 0/1 - charging MOSFET (reserved)
const uint8_t REG_DISCHARGE_SW = 0xAC; // 0/1 - discharging MOSFET

const unsigned long DEBOUNCE_MS = 50;
const unsigned long REFRESH_MS = 5000; // periodic resend of state
// Вбудований світлодіод. У режимі heartbeat світиться половину часу і тягне
// 0.5-1.5 мА — менше піввідсотка бюджету. Вимкнений заради економії,
// ціною єдиної зовнішньої ознаки, що прошивка жива.
// Світлодіод живлення на платі цим не гаситься: він припаяний до шини
// і знімається лише фізично.
#define HEARTBEAT_ENABLED 0
const unsigned long HEARTBEAT_INTERVAL_MS = 500;

// ===== Налаштування порогів індикації (налаштовувані константи) =====
const uint8_t SOC_LOW_THRESHOLD_PERCENT =
    20; // ЗЕЛЕНИЙ: понад цей заряд (%) горить рівно, на ньому й нижче мигає
const unsigned long LED_BLINK_INTERVAL_MS =
    500; // Інтервал мигання зеленого і червоного (мс)

const unsigned long BMS_TIMEOUT_MS = 5000; // Таймаут зв'язку з BMS (мс)
const unsigned long NO_CONN_STEP_MS = 300; // Крок біжучого вогню при втраті зв'язку (мс)

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
const char *ledWeb = "nc";         // Короткий код стану для веба

// Рядки інтерфейсу. Будуються один раз і йдуть і на екран, і у веб,
// щоб сторінка показувала рівно те саме, що й дисплей.
char uiLine[4][40] = {"", "", "", ""};

// Хто керує ключем. За умовчанням — фізичний тумблер; веб перехоплює
// керування лише коли на сторінці явно поставлена галочка.
bool webControlEnabled = false;

#if WIFI_AP_ENABLED
WebServer server(80);
bool apUp = false;
#endif
float bmsPackVoltageV = 0.0f;      // Напруга збірки, тег 0x83
float bmsCapacityAh = 0.0f;        // Ємність збірки, тег 0xAA
float remoteBatteryV = 0.0f;       // Власний акумулятор пульта
uint32_t remoteBatteryRawMv = 0;   // Сире значення на піні, для діагностики
unsigned long statusOverlayUntilMs = 0; // До якого моменту показувати ARM/DISARM
bool remoteCharging = false;       // Визначається за стрибком напруги
float remoteRatePctPerH = 0.0f;    // Швидкість зміни заряду пульта, %/год
bool remoteRateValid = false;      // Чи накопичилось даних для оцінки

// Корпус альбомний, модуль стоїть прямо: без повороту, полотно 128 x 64.
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
bool oledPresent = false;          // Чи відгукнувся дисплей на шині
int oledSdaIdle = -1, oledSclIdle = -1; // Рівні шини при останній спробі
uint8_t oledFailCount = 0;         // Поспіль невдалих/задовгих оновлень

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
#if !HEARTBEAT_ENABLED
  return; // світлодіод лишається погашеним із setupLeds()
#else
  static unsigned long lastHeartbeatMs = 0;
  static bool heartbeatState = false;

  if (millis() - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = millis();
    heartbeatState = !heartbeatState;
    setOnboardLed(heartbeatState);
  }
#endif
}

// Втрата зв'язку з BMS: біжучий вогонь червоний -> жовтий -> зелений.
// Послідовно, а не всі разом: три лампи, що мигають одночасно, бічним
// зором читаються як одна блимаюча, а біжучий вогонь ні з чим не
// сплутати — видно напрямок, тож стан очевидний і краєм ока.
void updateNoConnectionBlink() {
  static unsigned long lastStepMs = 0;
  static uint8_t phase = 0;

  if (millis() - lastStepMs >= NO_CONN_STEP_MS) {
    lastStepMs = millis();
    phase = (uint8_t)((phase + 1) % 3);
  }

  digitalWrite(LED_PIN_RED, phase == 0 ? HIGH : LOW);
  digitalWrite(LED_PIN_YELLOW, phase == 1 ? HIGH : LOW);
  digitalWrite(LED_PIN_GREEN, phase == 2 ? HIGH : LOW);
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

  // Немає зв'язку — біжучий вогонь, це має пріоритет над усім іншим
  if (!bmsConnected) {
    updateNoConnectionBlink();
    ledState = "CHASE R-Y-G (no link)";
    ledWeb = "nc";
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
    ledWeb = discharging ? "r" : "rb";
  } else if (discharging) {
    // Аварій немає, розряд активний: понад поріг — рівно, на порозі й нижче — мигає
    green = (bmsSocPercent > SOC_LOW_THRESHOLD_PERCENT) ? true : blinkState;
    ledState = (bmsSocPercent > SOC_LOW_THRESHOLD_PERCENT)
                   ? "GREEN solid (running)"
                   : "GREEN blink (running, SOC low)";
    ledWeb = (bmsSocPercent > SOC_LOW_THRESHOLD_PERCENT) ? "g" : "gb";
  } else {
    // Аварій немає, розряд вимкнений: батарея готова
    yellow = true;
    ledState = "YELLOW solid (ready)";
    ledWeb = "y";
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
      uint16_t rawPackV = ((uint16_t)buf[k + 1] << 8) | buf[k + 2];
      bmsPackVoltageV = (float)rawPackV * 0.01f;
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
    // Тег 0xAA: Налаштована ємність збірки (А·год) - 4 байти
    else if (tag == 0xAA && k + 4 < len - 4) {
      uint32_t rawCap = ((uint32_t)buf[k + 1] << 24) |
                        ((uint32_t)buf[k + 2] << 16) |
                        ((uint32_t)buf[k + 3] << 8) | buf[k + 4];
      if (rawCap > 0 && rawCap < 10000) bmsCapacityAh = (float)rawCap;
      k += 5;
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

// ===== OLED =====
// Дисплей навмисно НЕ є обовязковим: якщо він не відповів на шині або
// підвис під перешкодою, керування розрядом має працювати далі.
// Якщо раб завис посеред передачі, він утримує SDA, доки не дотягне свій
// байт. Стандартне звільнення: до девяти тактів на SCL, щоб він добив
// байт і відпустив лінію, потім STOP — перехід SDA з 0 в 1 при високому
// SCL. Все вручну, бо Wire на зайнятій шині не стартує.
bool recoverI2cBus() {
  pinMode(OLED_SDA_PIN, INPUT_PULLUP);
  pinMode(OLED_SCL_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(OLED_SCL_PIN, HIGH);
  delayMicroseconds(10);

  uint8_t pulses = 0;
  while (digitalRead(OLED_SDA_PIN) == LOW && pulses < 9) {
    digitalWrite(OLED_SCL_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(OLED_SCL_PIN, HIGH);
    delayMicroseconds(10);
    pulses++;
  }

  // STOP
  pinMode(OLED_SDA_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(OLED_SDA_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(OLED_SCL_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(OLED_SDA_PIN, HIGH);
  delayMicroseconds(10);

  pinMode(OLED_SDA_PIN, INPUT_PULLUP);
  pinMode(OLED_SCL_PIN, INPUT_PULLUP);
  delay(5);

  bool freed = (digitalRead(OLED_SDA_PIN) == HIGH);
  Serial.printf("OLED: bus recovery, %u pulses -> SDA %s\n", pulses,
                freed ? "released" : "still low");
  return freed;
}

bool probeOledAddress(uint8_t addr7) {
  Wire.beginTransmission(addr7);
  return Wire.endTransmission() == 0;
}

// Якщо дисплея немає або шина притиснута, сюди не можна заходити взагалі:
// Wire на зайнятій шині може блокуватись, а це підвісило б керування розрядом.
void setupOled() {
  Serial.println("OLED: checking bus...");

  pinMode(OLED_SDA_PIN, INPUT_PULLUP);
  pinMode(OLED_SCL_PIN, INPUT_PULLUP);
  delay(5);
  int sdaIdle = digitalRead(OLED_SDA_PIN);
  int sclIdle = digitalRead(OLED_SCL_PIN);
  oledSdaIdle = sdaIdle;
  oledSclIdle = sclIdle;
  Serial.printf("OLED: idle levels SDA=%d SCL=%d\n", sdaIdle, sclIdle);

  if (sdaIdle == LOW || sclIdle == LOW) {
    // Перш ніж здаватися, пробуємо розчепити шину
    if (!recoverI2cBus()) {
      Serial.println("OLED: bus still held low - skipping I2C entirely");
      return;
    }
    oledSdaIdle = digitalRead(OLED_SDA_PIN);
    oledSclIdle = digitalRead(OLED_SCL_PIN);
  }

  Wire.end(); // на випадок повторної спроби
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN, OLED_I2C_HZ);
  Wire.setTimeOut(50);
  Serial.println("OLED: Wire started, probing...");

  uint8_t addr = 0;
  if (probeOledAddress(0x3C)) addr = 0x3C;
  else if (probeOledAddress(0x3D)) addr = 0x3D;
  Serial.printf("OLED: probe done, addr=0x%02X\n", addr);

  if (addr == 0) {
    Wire.end();
    Serial.println("OLED: not found, running without display");
    return;
  }

  oled.setI2CAddress(addr << 1);
  Serial.println("OLED: begin()...");
  oled.begin();
  oled.setBusClock(OLED_I2C_HZ);
  oledPresent = true;
  Serial.printf("OLED: SH1106 ready at 0x%02X (SDA=GPIO%u SCL=GPIO%u)\n", addr,
                OLED_SDA_PIN, OLED_SCL_PIN);
}

// Відсоток заряду Li-Po за напругою. Крива розряду нелінійна: середина пласка,
// тому лінійне відображення 3.0-4.2 В брехало б на десятки відсотків.
// Таблиця — напруга спокою; під навантаженням показання занижене, одразу
// після зарядки — завищене, доки банка не осяде.
uint8_t liPoPercent(float v) {
  // Низ таблиці — звичайна крива Li-Po і не рухається. Верх підігнаний
  // під цей пульт: зарядка гасне на 4.16 В, а не на паспортних 4.20,
  // тож інакше повний акумулятор показувався б як 96 %.
  static const float tabV[] = {3.00f, 3.30f, 3.50f, 3.60f, 3.70f,
                               3.80f, 3.90f, 4.00f, 4.08f, 4.16f};
  static const uint8_t tabP[] = {0, 5, 10, 20, 35, 50, 65, 80, 90, 100};
  const uint8_t n = sizeof(tabP);

  if (v <= tabV[0]) return 0;
  if (v >= tabV[n - 1]) return 100;
  for (uint8_t i = 1; i < n; i++) {
    if (v <= tabV[i]) {
      float f = (v - tabV[i - 1]) / (tabV[i] - tabV[i - 1]);
      return (uint8_t)(tabP[i - 1] + f * (tabP[i] - tabP[i - 1]) + 0.5f);
    }
  }
  return 100;
}

// Прогноз часу роботи: скільки лишилось ампер-годин, поділити на струм.
// Має сенс лише під навантаженням; у спокої струм близький до нуля
// і будь-яке число було б вигадкою.
bool runtimeEstimate(uint16_t *hours, uint8_t *mins) {
  if (bmsCapacityAh <= 0.0f) return false;
  float load = fabs(bmsCurrentA);
  if (load < RUNTIME_MIN_CURRENT_A) return false;

  // При розряді рахуємо до порожнього, при заряді — до повного.
  float pct = (bmsCurrentA > 0.0f) ? (100.0f - (float)bmsSocPercent)
                                   : (float)bmsSocPercent;
  float remainAh = bmsCapacityAh * pct / 100.0f;
  float h = remainAh / load;
  if (h > 999.0f) h = 999.0f;
  *hours = (uint16_t)h;
  *mins = (uint8_t)((h - (float)*hours) * 60.0f + 0.5f);
  if (*mins >= 60) { *mins = 0; (*hours)++; }
  return true;
}

// "Розряд 8г 20хв", "Заряд >99г", "Спокій --" — спільний формат.
// Відємне h означає, що оцінити неможливо.
void formatRuntime(char *out, size_t n, const char *label, float h) {
  if (h < 0.0f) {
    snprintf(out, n, "%s --", label);
    return;
  }
  if (h > 99.0f) {
    snprintf(out, n, "%s >99г", label);
    return;
  }
  uint16_t hh = (uint16_t)h;
  uint8_t mm = (uint8_t)((h - (float)hh) * 60.0f + 0.5f);
  if (mm >= 60) { mm = 0; hh++; }
  snprintf(out, n, "%s %uг %02uхв", label, (unsigned)hh, (unsigned)mm);
}

// Текст аварії: назва плюс значення, що її викликало. Без числа напис
// каже лише "щось не так", а оператору треба знати наскільки.
// Рядок має вкладатись у 16 символів — стільки тримає шрифт 7x13.
void faultText(uint8_t code, char *out, size_t n) {
  switch (code) {
    case 1: {
      // Показуємо ту межу, яку пробито: занизьку чи зависоку
      float v = (bmsMinCellV < 2.60f) ? bmsMinCellV : bmsMaxCellV;
      snprintf(out, n, "Комірка %.2fВ", v);
      break;
    }
    case 2: snprintf(out, n, "Струм %.0fA", fabs(bmsCurrentA)); break;
    case 3: snprintf(out, n, "Температура %.0fC", bmsTempC); break;
    case 4: snprintf(out, n, "MOSFET %.0fC", bmsTempC); break;
    case 5: snprintf(out, n, "КЗ %.0fA", fabs(bmsCurrentA)); break;
    case 6:
      snprintf(out, n, "Баланс %.0fмВ", bmsCellDeltaV * 1000.0f);
      break;
    default: out[0] = 0; break;
  }
}

// Будує чотири рядки інтерфейсу. Викликається з циклу незалежно від того,
// чи є дисплей: ці самі рядки віддаються у веб.
// Єдине місце, де змінюється бажаний стан ключа — і з тумблера, і з вебу.
void applyDischargeCommand(bool on, const char *source) {
  targetDischargeState = on;
  statusOverlayUntilMs = millis() + STATUS_OVERLAY_MS;
  jkWriteRegister(REG_DISCHARGE_SW, on ? 0x01 : 0x00);
  lastWriteRetryMs = millis();
  Serial.printf(">>> DISCHARGE %s (%s) <<<\n", on ? "ON" : "OFF", source);
}

// Друкується за таймером, незалежно від того, чи відповідає BMS.
// Раніше рядок виходив лише при розборі кадру, тож обрив зв язку
// гасив увесь лог — рівно в той момент, коли він найпотрібніший.
void printStatus() {
        Serial.printf(
            "BMS Status: Pack = %.2f V | Current = %.2f A | SOC = %d%% | Temp = %.1f C | MinV = %.3f V | MaxV = %.3f V | DeltaV = %.3f V | Cap = %.0fAh | DSG = %s | OLED = %s(%d%d) | AP = %s | PULT = %.2f V (%lu mV, %s, %+.1f%%/h) | LED: %s\n",
            bmsPackVoltageV, bmsCurrentA, bmsSocPercent, bmsTempC, bmsMinCellV, bmsMaxCellV, bmsCellDeltaV,
            bmsCapacityAh, bmsActualDischargeState ? "ON " : "OFF",
            oledPresent ? "ok" : "--", oledSdaIdle, oledSclIdle,
#if WIFI_AP_ENABLED
            apUp ? WiFi.softAPIP().toString().c_str() : "down",
#else
            "off",
#endif
            remoteBatteryV,
            (unsigned long)remoteBatteryRawMv,
            remoteCharging ? "zaryad" : "rozryad",
            remoteRateValid ? remoteRatePctPerH : 0.0f, ledState);
}
void buildUiLines() {
  char buf[40];

  uint8_t pultPct = liPoPercent(remoteBatteryV);
  snprintf(uiLine[0], sizeof(uiLine[0]), "Пульт %u%% %.2fВ", (unsigned)pultPct,
           remoteBatteryV);

  const char *pultLbl = remoteCharging ? "Заряд" : "Розряд";
  float pultH = -1.0f;
  if (remoteRateValid && remoteRatePctPerH > RATE_MIN_PCT_PER_H) {
    pultH = (100.0f - (float)pultPct) / remoteRatePctPerH;
  } else if (remoteRateValid && remoteRatePctPerH < -RATE_MIN_PCT_PER_H) {
    pultH = (float)pultPct / -remoteRatePctPerH;
  } else if (!remoteCharging && remoteBatteryV > 0.0f) {
    float mah = (float)PULT_CAPACITY_MAH * (float)pultPct / 100.0f;
    pultH = mah / (float)PULT_TYPICAL_DRAW_MA;
  }
  formatRuntime(uiLine[1], sizeof(uiLine[1]), pultLbl, pultH);

  if (bmsConnected) {
    snprintf(uiLine[2], sizeof(uiLine[2]), "РЕБ %u%% %.1fA",
             (unsigned)bmsSocPercent, bmsCurrentA);
  } else {
    snprintf(uiLine[2], sizeof(uiLine[2]), "РЕБ --");
  }

  uint8_t f = bmsConnected ? determineActiveFaultCode() : 0;
  if (f != 0) {
    faultText(f, uiLine[3], sizeof(uiLine[3]));
  } else if (bmsConnected) {
    const char *lbl = (bmsCurrentA > RUNTIME_MIN_CURRENT_A)    ? "Заряд"
                      : (bmsCurrentA < -RUNTIME_MIN_CURRENT_A) ? "Розряд"
                                                               : "Спокій";
    uint16_t rh;
    uint8_t rm;
    if (runtimeEstimate(&rh, &rm)) {
      formatRuntime(buf, sizeof(buf), lbl, (float)rh + (float)rm / 60.0f);
    } else {
      formatRuntime(buf, sizeof(buf), lbl, -1.0f);
    }
    snprintf(uiLine[3], sizeof(uiLine[3]), "%s", buf);
  } else {
    // Лампи блимають, але екран мовчав про причину. Треба назвати словами:
    // мигання само по собі не каже, що саме сталося.
    snprintf(uiLine[3], sizeof(uiLine[3]), "Немає зв'язку");
  }
}

void updateOled() {
  if (!oledPresent) return;

  static unsigned long lastDrawMs = 0;
  if (millis() - lastDrawMs < OLED_REFRESH_MS) return;
  lastDrawMs = millis();

  unsigned long t0 = millis();
  oled.clearBuffer();

  const uint8_t x0 = OLED_MARGIN_X;
  const uint8_t y0 = OLED_MARGIN_Y;
  const uint8_t w = oled.getDisplayWidth() - 2 * OLED_MARGIN_X;

  if (millis() < statusOverlayUntilMs) {
    const char *word = bmsActualDischargeState ? "Armed" : "Disarmed";
    oled.setFont(u8g2_font_10x20_tr);
    uint8_t tw = oled.getStrWidth(word);
    oled.drawStr(x0 + (w - tw) / 2, y0 + 36, word);
  } else {
    oled.setFont(u8g2_font_7x13_t_cyrillic);
    for (uint8_t i = 0; i < 4; i++) {
      oled.drawUTF8(x0, y0 + 11 + i * 13, uiLine[i]);
    }
  }

  oled.sendBuffer();
  unsigned long took = millis() - t0;

  // Захист від зависання шини: не мовчимо, але й не тягнемо цикл за собою
  if (took > OLED_RENDER_BUDGET_MS) {
    if (++oledFailCount >= OLED_MAX_FAILS) {
      oledPresent = false;
      Serial.printf("OLED: disabled after %u slow renders (last %lu ms). "
                    "Control keeps running.\n",
                    oledFailCount, took);
    }
  } else if (oledFailCount) {
    oledFailCount = 0;
  }
}

// Власний акумулятор пульта. Читається з усередненням: поруч два імпульсні
// перетворювачі, і одиночна вибірка стрибає.
void updateRemoteBattery() {
  static unsigned long lastMs = 0;
  if (remoteBatteryV > 0.0f && millis() - lastMs < VBAT_PERIOD_MS) return;
  lastMs = millis();

  uint32_t acc = 0;
  for (uint8_t i = 0; i < VBAT_SAMPLES; i++) {
    acc += analogReadMilliVolts(PIN_VBAT_SENSE);
  }
  remoteBatteryRawMv = acc / VBAT_SAMPLES;
  remoteBatteryV =
      (float)remoteBatteryRawMv * VBAT_DIVIDER * VBAT_CALIBRATION / 1000.0f;

  // Головний детектор: стрибок між сусідніми вимірами. Підключення або
  // відєднання зарядки видно одразу, за один період.
  static float prevV = 0.0f;
  bool stepSeen = false;
  if (prevV > 0.0f) {
    float step = remoteBatteryV - prevV;
    if (step > STEP_THRESHOLD_V) {
      remoteCharging = true;
      stepSeen = true;
    } else if (step < -STEP_THRESHOLD_V) {
      remoteCharging = false;
      stepSeen = true;
    }
  }
  prevV = remoteBatteryV;

  // Швидкість зміни заряду. Струм для цього не потрібен: відсоток ми
  // вже рахуємо з кривої, час знаємо, отже похідна спостережна.
  {
    uint8_t pctNow = liPoPercent(remoteBatteryV);
    unsigned long nowMs = millis();

    static uint8_t histPct[RATE_SLOTS];
    static unsigned long histMs[RATE_SLOTS];
    static uint8_t histCount = 0, histHead = 0;
    static unsigned long lastPushMs = 0;
    static float rateRing[RATE_AVG_SLOTS];
    static uint8_t rateCount = 0, rateHead = 0;

    // Підключення або відєднання зарядки — зовнішня дія, а не заряд.
    // Історію після неї треба починати заново, інакше перепад у 230 мВ
    // дасть фантастичну швидкість.
    if (stepSeen) {
      histCount = histHead = 0;
      rateCount = rateHead = 0;
      lastPushMs = 0;
      remoteRateValid = false;
      remoteRatePctPerH = 0.0f;
    }

    if (lastPushMs == 0 || nowMs - lastPushMs >= RATE_SAMPLE_MS) {
      lastPushMs = nowMs;
      histPct[histHead] = pctNow;
      histMs[histHead] = nowMs;
      histHead = (uint8_t)((histHead + 1) % RATE_SLOTS);
      if (histCount < RATE_SLOTS) histCount++;

      // Найстаріший зразок у вікні. Вікно ковзає, тож швидкість міняється
      // плавно, без стрибка на межі.
      // Тільки на повному вікні: на частковому крок квантування відсотка
      // перетворюється на фантомний заряд. Ціна — перші 5 хвилин після
      // вмикання швидкості немає, і час береться з моделі.
      if (histCount >= RATE_SLOTS) {
        uint8_t oldest =
            (uint8_t)((histHead + RATE_SLOTS - histCount) % RATE_SLOTS);
        unsigned long span = nowMs - histMs[oldest];
        float hours = (float)span / 3600000.0f;
        float raw = ((float)pctNow - (float)histPct[oldest]) / hours;

        rateRing[rateHead] = raw;
        rateHead = (uint8_t)((rateHead + 1) % RATE_AVG_SLOTS);
        if (rateCount < RATE_AVG_SLOTS) rateCount++;

        float sum = 0.0f;
        for (uint8_t i = 0; i < rateCount; i++) sum += rateRing[i];
        remoteRatePctPerH = sum / (float)rateCount;
        remoteRateValid = true;

        // Знак усередненої швидкості — ознака напрямку. Працює й тоді,
        // коли пульт увімкнули з уже підключеною зарядкою.
        if (remoteRatePctPerH > RATE_MIN_PCT_PER_H) remoteCharging = true;
        else if (remoteRatePctPerH < -RATE_MIN_PCT_PER_H)
          remoteCharging = false;
      }
    }
  }
#if PULT_CHARGE_PIN_ENABLED
  // Апаратний сигнал має пріоритет над здогадкою: CHRG у нулі = йде заряд
  remoteCharging = (digitalRead(PIN_PULT_CHARGE) == LOW);
#endif
}

#if WIFI_AP_ENABLED
static const char PAGE_HTML[] PROGMEM = R"HTMLPAGE(
<!doctype html><html lang="uk"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Пульт РЕБ Хвиля</title><style>
body{margin:0;background:#11171a;color:#e7edeb;
font:15px/1.5 system-ui,Segoe UI,Roboto,sans-serif;padding:18px}
.wrap{max-width:420px;margin:0 auto}
h1{font-size:17px;margin:0 0 16px;color:#85989d;font-weight:600;
letter-spacing:.08em;text-transform:uppercase}
.lamps{display:flex;gap:18px;justify-content:center;margin:0 0 20px}
.lamp{width:62px;height:62px;border-radius:50%;background:#1b2427;
border:2px solid #2a373b;transition:background .15s}
.lamp.on.r{background:#d14026;box-shadow:0 0 22px #d1402688}
.lamp.on.y{background:#d9a81c;box-shadow:0 0 22px #d9a81c88}
.lamp.on.g{background:#2fa360;box-shadow:0 0 22px #2fa36088}
.lamp.blink{animation:b .6s steps(1,end) infinite}
.lamp.chase{animation:c .9s steps(1,end) infinite}
#ly.chase{animation-delay:.3s}
#lg.chase{animation-delay:.6s}
@keyframes b{50%{background:#1b2427;box-shadow:none}}
@keyframes c{33%{background:#1b2427;box-shadow:none}}
.rows{background:#161d20;border:1px solid #2a373b;border-radius:6px;
padding:12px 14px;margin:0 0 18px}
.rows div{font-family:ui-monospace,Consolas,monospace;font-size:16px;
padding:3px 0}
.ctl{background:#161d20;border:1px solid #2a373b;border-radius:6px;
padding:14px}
label{display:flex;align-items:center;gap:10px;cursor:pointer;
margin:0 0 12px}
input[type=checkbox]{width:20px;height:20px;accent-color:#d9a81c}
button{width:100%;padding:14px;font-size:16px;font-weight:700;
border:0;border-radius:6px;cursor:pointer;color:#11171a}
button:disabled{opacity:.35;cursor:not-allowed}
.arm{background:#2fa360}.disarm{background:#d14026;color:#fff}
.hint{color:#85989d;font-size:13px;margin:10px 0 0}
</style></head><body><div class="wrap">
<h1>Пульт РЕБ Хвиля</h1>
<div class="lamps">
<div id="lr" class="lamp r"></div>
<div id="ly" class="lamp y"></div>
<div id="lg" class="lamp g"></div></div>
<div class="rows"><div id="r0"></div><div id="r1"></div>
<div id="r2"></div><div id="r3"></div></div>
<div class="ctl">
<label><input type="checkbox" id="wc">
<span>Забрати керування</span></label>
<button id="bt" disabled></button>
<p class="hint" id="hint"></p>
</div></div><script>
let dsg=0,web=0;
function ap(j){
for(let i=0;i<4;i++)document.getElementById("r"+i).textContent=j.l[i];
const m={r:["lr",0],rb:["lr",1],y:["ly",0],g:["lg",0],gb:["lg",1]};
["lr","ly","lg"].forEach(k=>{const e=document.getElementById(k);
e.className="lamp "+k[1]});
if(j.led=="nc"){["lr","ly","lg"].forEach(k=>{const e=
document.getElementById(k);e.className="lamp "+k[1]+" on chase"})}
else if(m[j.led]){const e=document.getElementById(m[j.led][0]);
e.className="lamp "+m[j.led][0][1]+" on"+(m[j.led][1]?" blink":"")}
dsg=j.dsg;web=j.web;
document.getElementById("wc").checked=!!web;
const b=document.getElementById("bt");
b.disabled=!web;
b.textContent=dsg?"Вимкнути розряд":"Увімкнути розряд";
b.className=dsg?"disarm":"arm";
document.getElementById("hint").textContent=web?
"Тумблер ігнорується, поки галочка стоїть.":
"Керує фізичний тумблер.";}
function poll(){fetch("/api").then(r=>r.json()).then(ap).catch(()=>{})}
document.getElementById("wc").onchange=e=>{
fetch("/set?web="+(e.target.checked?1:0)).then(poll)};
document.getElementById("bt").onclick=()=>{
fetch("/set?dsg="+(dsg?0:1)).then(poll)};
poll();setInterval(poll,1000);
</script></body></html>
)HTMLPAGE";

void handleRoot() { server.send_P(200, "text/html; charset=utf-8", PAGE_HTML); }

void handleApi() {
  char js[420];
  snprintf(js, sizeof(js),
           "{\"l\":[\"%s\",\"%s\",\"%s\",\"%s\"],"
           "\"led\":\"%s\",\"dsg\":%d,\"link\":%d,\"web\":%d}",
           uiLine[0], uiLine[1], uiLine[2], uiLine[3], ledWeb,
           bmsActualDischargeState ? 1 : 0, bmsConnected ? 1 : 0,
           webControlEnabled ? 1 : 0);
  server.send(200, "application/json; charset=utf-8", js);
}

void handleSet() {
  if (server.hasArg("web")) {
    webControlEnabled = (server.arg("web") == "1");
    Serial.printf("WEB: control %s\n",
                  webControlEnabled ? "taken by browser" : "back to switch");
  }
  // Команду приймаємо лише коли керування явно передане вебу.
  if (server.hasArg("dsg") && webControlEnabled) {
    applyDischargeCommand(server.arg("dsg") == "1", "web");
  }
  server.send(200, "text/plain", "ok");
}

void setupWifi() {
  WiFi.mode(WIFI_AP);
  // Нижча потужність: менше струму і менше помітності в ефірі.
  WiFi.setTxPower(WIFI_POWER_11dBm);
  apUp = WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, AP_MAX_CLIENTS);
  if (!apUp) {
    Serial.println("WIFI: softAP failed, running without web");
    return;
  }
  server.on("/", handleRoot);
  server.on("/api", handleApi);
  server.on("/set", handleSet);
  server.begin();
  Serial.printf("WIFI: AP \"%s\" up at %s\n", AP_SSID,
                WiFi.softAPIP().toString().c_str());
}
#endif

void setup() {
  Serial.begin(115200);
  delay(2000); // час хосту підхопити USB CDC після скидання
  pinMode(PIN_BUTTON, INPUT_PULLDOWN);
  analogSetPinAttenuation(PIN_VBAT_SENSE, ADC_11db); // повний діапазон ~0-3.1 V
#if PULT_CHARGE_PIN_ENABLED
  pinMode(PIN_PULT_CHARGE, INPUT_PULLUP); // CHRG — відкритий колектор
#endif // switch pulls the pin UP to 3.3 V
  setupLeds();

  Serial.println();
  Serial.println("JK BMS controller starting");
  Serial1.begin(115200, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
  Serial.println("Serial1 up");
  setupOled();
#if WIFI_AP_ENABLED
  setupWifi();
#endif
  Serial.println("setup() done");
  Serial.printf("Serial1 initialized: RX=GPIO%d, TX=GPIO%d\n", BMS_RX_PIN,
                BMS_TX_PIN);
  delay(300);

  lastRawPressed = lastStablePressed =
      (digitalRead(PIN_BUTTON) == BUTTON_ACTIVE_LEVEL);
  setDischarge(lastStablePressed);
  lastRefreshMs = millis();
}

void loop() {
#if WIFI_AP_ENABLED
  if (apUp) server.handleClient();
#endif
  processBmsResponses();

  updateHeartbeat();
  updateLedIndicators();
  updateRemoteBattery();

  static unsigned long lastUiMs = 0;
  if (millis() - lastUiMs >= 250) {
    lastUiMs = millis();
    buildUiLines();
  }

  static unsigned long lastStatusMs = 0;
  if (millis() - lastStatusMs >= 1000) {
    lastStatusMs = millis();
    printStatus();
  }

  // Дисплей міг не відповісти на старті або відвалитися пізніше.
  // Пробуємо знайти його знову, щоб він піднявся без перезавантаження.
  if (!oledPresent) {
    static unsigned long lastProbeMs = 0;
    if (millis() - lastProbeMs >= 5000) {
      lastProbeMs = millis();
      setupOled();
    }
  }
  updateOled();

  bool raw = (digitalRead(PIN_BUTTON) == BUTTON_ACTIVE_LEVEL);
  unsigned long now = millis();

  if (raw != lastRawPressed) {
    lastRawPressed = raw;
    lastEdgeMs = now;
  }

  // Положення тумблера відстежуємо завжди, щоб знати його навіть тоді,
  // коли керує веб: при поверненні керування треба буде до нього прийти.
  bool switchMoved = false;
  if ((now - lastEdgeMs) >= DEBOUNCE_MS && raw != lastStablePressed) {
    lastStablePressed = raw;
    switchMoved = true;
  }

  if (!webControlEnabled) {
    // Звичайний режим: ключ повторює положення тумблера. Сюди ж
    // потрапляємо одразу після зняття галочки — і стан вирівнюється
    // по тумблеру, без окремої обробки.
    if (switchMoved || targetDischargeState != lastStablePressed) {
      applyDischargeCommand(lastStablePressed, "switch");
    }
  }
  // Якщо керує веб — тумблер лише відстежуємо, команд з нього не беремо.

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
  bool awaitingConfirm = bmsConnected &&
                         (bmsActualDischargeState != targetDischargeState);
  unsigned long telemetryPeriod =
      (awaitingConfirm || now < statusOverlayUntilMs) ? TELEMETRY_FAST_MS
                                                      : TELEMETRY_IDLE_MS;
  if ((now - lastTelemetryReqMs) >= telemetryPeriod) {
    lastTelemetryReqMs = now;
    jkRequestTelemetry();
  }
}
