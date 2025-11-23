#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ------- CONFIG: ESPNOW / FRIDGE CONTROL -------
static const uint8_t CONTROLLER_MAC[6] = {0x24, 0x0A, 0xC4, 0xAA, 0xBB, 0xCC};

const float   SETPOINT    = 3.0;
const float   ON_DELTA    = 1.0;
const float   OFF_DELTA   = 0.5;
const uint32_t MIN_ON_MS  = 60000;
const uint32_t MIN_OFF_MS = 60000;

const int ESPNOW_CH = 6;

// ------------ I2C PINS (ESP32) ------------
#define I2C_SDA 21
#define I2C_SCL 22

// ------------ LCD SETUP (New-LiquidCrystal) ------------
// addr, En, Rw, Rs, d4, d5, d6, d7, backlight, backlight polarity
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// ------------ ONE-WIRE BUS ------------
#define ONE_WIRE_PIN 13
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature ds18b20(&oneWire);

// ------------ SENSOR ADDRESSES (from your scan) ------------
// Sensor 0: 280088780000005F
// Sensor 1: 28BC237800000066
// Sensor 2: 28A9AF780000003C

DeviceAddress addrFridge = { 0x28, 0x00, 0x88, 0x78, 0x00, 0x00, 0x00, 0x5F }; // fridge temp
DeviceAddress addrCold   = { 0x28, 0xBC, 0x23, 0x78, 0x00, 0x00, 0x00, 0x66 }; // cold-side coolant
DeviceAddress addrHot    = { 0x28, 0xA9, 0xAF, 0x78, 0x00, 0x00, 0x00, 0x3C }; // hot-side coolant

// --- ESP-NOW packet structures ---
struct TxPacket {
  uint32_t seq;
  uint32_t millis32;
  int16_t  t_box;
  int16_t  t_hot;
  int16_t  t_amb;
  int16_t  t_cool;
  uint8_t  fridge_on;
  uint8_t  flags;
  uint16_t crc16;
} __attribute__((packed));

struct RxAck {
  uint32_t ack_seq;
  int8_t   rssi;
  uint8_t  outputs;
  uint8_t  health;
  uint16_t crc16;
} __attribute__((packed));

volatile bool gotAck = false;
RxAck lastAck{};
int8_t  lastRssi = 0;
uint32_t lastAckMs = 0;

// --- CRC16 helper ---
uint16_t crc16(const uint8_t* d, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int j = 0; j < 8; j++) {
      c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
    }
  }
  return c;
}

// --- Simple validity helper (optional) ---
bool deviceAddressIsValid(const DeviceAddress addr) {
  bool allZero = true, allFF = true;
  for (int i = 0; i < 8; i++) {
    if (addr[i] != 0x00) allZero = false;
    if (addr[i] != 0xFF) allFF  = false;
  }
  return !(allZero || allFF);
}

// === ESP-NOW callbacks (IDF v5 style) ===

// Send callback: now gets wifi_tx_info_t* instead of MAC
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  (void)status;
}

// Receive callback: now gets esp_now_recv_info* and data
void onDataRecv(const esp_now_recv_info *info, const uint8_t* data, int len) {
  (void)info; // not used right now
  if (len == (int)sizeof(RxAck)) {
    memcpy(&lastAck, data, len);
    gotAck   = true;
    lastAckMs = millis();
  }
}

// --- Control state ---
bool fridgeOn = false;
uint32_t lastToggle = 0;
uint32_t seq = 0;

inline int16_t s10_from_float(float c) {
  return isfinite(c) ? (int16_t)round(c * 10.0f) : (int16_t)-32768;
}

String fmtTemp(float v) {
  if (v == DEVICE_DISCONNECTED_C || !isfinite(v)) return String("--.-");
  char buf[8];
  sprintf(buf, "%5.1f", v);
  return String(buf);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // I2C + LCD init (EXACTLY as in your working code)
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.begin(20, 4);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Yacht Fridge: SENDER");
  lcd.setCursor(0, 1);
  lcd.print("Bus on pin 13");

  // DS18B20 init
  ds18b20.begin();
  if (deviceAddressIsValid(addrFridge)) ds18b20.setResolution(addrFridge, 12);
  if (deviceAddressIsValid(addrCold))   ds18b20.setResolution(addrCold,   12);
  if (deviceAddressIsValid(addrHot))    ds18b20.setResolution(addrHot,    12);

  int count = ds18b20.getDeviceCount();
  Serial.printf("Found %d DS18B20 sensors on pin 13\n", count);

  delay(1500);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Yacht Fridge: SENDER");

  // Wi-Fi / ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CH, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    lcd.setCursor(0, 1);
    lcd.print("ESP-NOW init fail");
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, CONTROLLER_MAC, 6);
  peer.channel = ESPNOW_CH;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
    lcd.setCursor(0, 1);
    lcd.print("Peer add failed   ");
  }
}

void loop() {
  // Ask all sensors to do a conversion
  ds18b20.requestTemperatures();

  float tFridge = ds18b20.getTempC(addrFridge);
  float tCold   = ds18b20.getTempC(addrCold);
  float tHot    = ds18b20.getTempC(addrHot);
  float tAmb    = DEVICE_DISCONNECTED_C; // no ambient probe yet

  bool sensorsOK   = (tFridge != DEVICE_DISCONNECTED_C) && (tHot != DEVICE_DISCONNECTED_C);
  bool alarmHot    = sensorsOK && (tHot > 65.0f);
  bool alarmProbe  = !sensorsOK;

  // Hysteresis & anti short-cycle
  uint32_t now = millis();
  if (!fridgeOn) {
    if (isfinite(tFridge) && tFridge > (SETPOINT + ON_DELTA) && (now - lastToggle) >= MIN_OFF_MS) {
      fridgeOn   = true;
      lastToggle = now;
    }
  } else {
    if (isfinite(tFridge) && tFridge < (SETPOINT - OFF_DELTA) && (now - lastToggle) >= MIN_ON_MS) {
      fridgeOn   = false;
      lastToggle = now;
    }
  }

  // Build & send packet
  TxPacket p{};
  p.seq       = ++seq;
  p.millis32  = now;
  p.t_box     = s10_from_float((tFridge == DEVICE_DISCONNECTED_C) ? NAN : tFridge);
  p.t_hot     = s10_from_float((tHot    == DEVICE_DISCONNECTED_C) ? NAN : tHot);
  p.t_amb     = s10_from_float((tAmb    == DEVICE_DISCONNECTED_C) ? NAN : tAmb);
  p.t_cool    = s10_from_float((tCold   == DEVICE_DISCONNECTED_C) ? NAN : tCold);
  p.fridge_on = fridgeOn ? 1 : 0;
  p.flags     = (sensorsOK ? 1 : 0) |
                (alarmHot   ? (1 << 1) : 0) |
                (alarmProbe ? (1 << 2) : 0);
  p.crc16 = crc16((uint8_t*)&p, sizeof(p) - 2);

  esp_now_send(CONTROLLER_MAC, (uint8_t*)&p, sizeof(p));

  // ---------- LCD DISPLAY ----------
  lcd.setCursor(0, 0);
  lcd.print("Box:");
  lcd.print(fmtTemp(tFridge));
  lcd.print("C ");

  lcd.setCursor(0, 1);
  lcd.print("Hot:");
  lcd.print(fmtTemp(tHot));
  lcd.print("C  ");

  lcd.setCursor(0, 2);
  lcd.print("Cold:");
  lcd.print(fmtTemp(tCold));
  lcd.print("C Fr:");
  lcd.print(fridgeOn ? "ON " : "OFF");

  lcd.setCursor(0, 3);
  uint32_t age = gotAck ? (now - lastAckMs) : 99999;
  lcd.print("ACK:");
  lcd.print(gotAck ? "OK " : "-- ");
  lcd.print(" Age:");
  lcd.print((unsigned long)(age / 1000));
  lcd.print("s   ");

  delay(1000);
}
