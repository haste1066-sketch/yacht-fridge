#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_now.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ------- CONFIG SNIPPET (inline for demo; mirror values from config.yaml) -------
static const uint8_t CONTROLLER_MAC[6] = {0x24,0x0A,0xC4,0xAA,0xBB,0xCC};
const int LCD_ADDR=0x27, LCD_COLS=20, LCD_ROWS=4, I2C_SDA=21, I2C_SCL=22; // All DS18B20 sensors share the same OneWire bus on GPIO 13
#define ONE_WIRE_BUS 13
// ROM strings (16 hex chars each) for the three DS18B20 sensors
const char *ROM_FRIDGE = "280088780000005F";   // fridge temp
const char *ROM_COLD   = "28BC237800000066";   // cold-side coolant temp (Peltier cold)
const char *ROM_HOT    = "28A9AF780000003C";  // hot-side coolant temp (Peltier hot)

const float SETPOINT=3.0, ON_DELTA=1.0, OFF_DELTA=0.5;
const uint32_t MIN_ON_MS=60000, MIN_OFF_MS=60000;
const int ESPNOW_CH=6;
// --------------------------------------------------------------------------------

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

struct TxPacket {
  uint32_t seq, millis32;
  int16_t t_box, t_hot, t_amb, t_cool;
  uint8_t fridge_on, flags;
  uint16_t crc16;
} __attribute__((packed));

struct RxAck {
  uint32_t ack_seq;
  int8_t rssi;
  uint8_t outputs, health;
  uint16_t crc16;
} __attribute__((packed));

volatile bool gotAck=false;
RxAck lastAck{};
int8_t lastRssi = 0;
uint32_t lastAckMs=0;

uint16_t crc16(const uint8_t* d, size_t n){
  uint16_t c=0xFFFF;
  for(size_t i=0;i<n;i++){ c ^= d[i]; for(int j=0;j<8;j++) c = (c&1)?(c>>1)^0xA001:(c>>1); }
  return c;
}

// OneWire / DallasTemperature setup
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

typedef uint8_t DeviceAddress8[8];
DeviceAddress8 addr_fridge;
DeviceAddress8 addr_cold;
DeviceAddress8 addr_hot;

// parse a 16-char hex string (like "28A9AF780000003C") into 8-byte device address
bool parseRomString(const char *hexStr, DeviceAddress8 addr) {
  if (!hexStr) return false;
  size_t len = strlen(hexStr);
  if (len != 16) return false;
  char byteStr[3] = {0,0,0};
  for (int i = 0; i < 8; ++i) {
    byteStr[0] = hexStr[i*2];
    byteStr[1] = hexStr[i*2 + 1];
    // strtol handles upper/lowercase hex
    addr[i] = (uint8_t)strtol(byteStr, nullptr, 16);
  }
  return true;
}

void printAddress(const DeviceAddress8 addr) {
  for (uint8_t i = 0; i < 8; i++) {
    if (addr[i] < 16) Serial.print('0');
    Serial.print(addr[i], HEX);
  }
}

void onDataSent(const uint8_t*, esp_now_send_status_t status){
  // status only shows TX result; ACK comes via onDataRecv
}

void onDataRecv(const uint8_t* mac, const uint8_t* data, int len){
  if(len==(int)sizeof(RxAck)){ 
    memcpy(&lastAck, data, len);
    gotAck=true;
    lastAckMs = millis();
  }
}

bool deviceAddressIsValid(const DeviceAddress8 addr) {
  // simple check: not all 0x00 or all 0xFF
  bool allZero=true, allFF=true;
  for(int i=0;i<8;i++){
    if(addr[i]!=0x00) allZero=false;
    if(addr[i]!=0xFF) allFF=false;
  }
  return !(allZero || allFF);
}

void setup(){
  Serial.begin(115200);
  delay(100);
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init(); lcd.backlight();
  lcd.clear(); lcd.setCursor(0,0); lcd.print("Yacht Fridge: SENDER");

  // parse ROMs
  if(!parseRomString(ROM_FRIDGE, addr_fridge)) Serial.println("Failed to parse fridge ROM");
  if(!parseRomString(ROM_COLD, addr_cold))   Serial.println("Failed to parse cold ROM");
  if(!parseRomString(ROM_HOT, addr_hot))     Serial.println("Failed to parse hot ROM");

  Serial.print("OneWire bus pin: ");
  Serial.println(ONE_WIRE_BUS);
  Serial.print("Fridge ROM: ");
  printAddress(addr_fridge); Serial.println();
  Serial.print("Cold ROM: ");
  printAddress(addr_cold); Serial.println();
  Serial.print("Hot ROM: ");
  printAddress(addr_hot); Serial.println();

  ds18b20.begin();
  // Optionally set resolution per-device
  if(deviceAddressIsValid(addr_fridge)) ds18b20.setResolution((uint8_t*)addr_fridge, 12);
  if(deviceAddressIsValid(addr_cold))   ds18b20.setResolution((uint8_t*)addr_cold, 12);
  if(deviceAddressIsValid(addr_hot))    ds18b20.setResolution((uint8_t*)addr_hot, 12);

  Serial.print("Devices found on bus: ");
  Serial.println(ds18b20.getDeviceCount());

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CH, WIFI_SECOND_CHAN_NONE);
  if(esp_now_init()!=ESP_OK){ lcd.setCursor(0,1); lcd.print("ESP-NOW init fail"); return; }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  esp_now_peer_info_t peer{}; memcpy(peer.peer_addr, CONTROLLER_MAC, 6);
  peer.channel=ESPNOW_CH; peer.encrypt=false;
  esp_now_add_peer(&peer);
}

bool fridgeOn=false;
uint32_t lastToggle=0;
uint32_t seq=0;

inline int16_t s10_from_float(float c){
  return isfinite(c)? (int16_t)round(c*10.0) : (int16_t)-32768;
}

void loop(){
  // request temps from all devices on the bus
  ds18b20.requestTemperatures();

  float tBox = DEVICE_DISCONNECTED_C;
  float tCold = DEVICE_DISCONNECTED_C;
  float tHot = DEVICE_DISCONNECTED_C;
  float tAmb = DEVICE_DISCONNECTED_C; // optional/unused if no ambient probe

  if(deviceAddressIsValid(addr_fridge)) tBox = ds18b20.getTempC((uint8_t*)addr_fridge);
  if(deviceAddressIsValid(addr_cold))   tCold = ds18b20.getTempC((uint8_t*)addr_cold);
  if(deviceAddressIsValid(addr_hot))    tHot = ds18b20.getTempC((uint8_t*)addr_hot);

  bool sensorsOK = (tBox != DEVICE_DISCONNECTED_C) && (tHot != DEVICE_DISCONNECTED_C);
  bool alarmHot  = sensorsOK && (tHot > 65.0);
  bool alarmProbe= !sensorsOK;

  // Hysteresis & anti short-cycle
  uint32_t now=millis();
  if(!fridgeOn){
    if(isfinite(tBox) && tBox > (SETPOINT+ON_DELTA) && (now-lastToggle)>=MIN_OFF_MS) { fridgeOn=true; lastToggle=now; }
  } else {
    if(isfinite(tBox) && tBox < (SETPOINT-OFF_DELTA) && (now-lastToggle)>=MIN_ON_MS) { fridgeOn=false; lastToggle=now; }
  }

  // Build & send packet
  TxPacket p{};
  p.seq = ++seq;
  p.millis32 = now;
  p.t_box  = s10_from_float((tBox==DEVICE_DISCONNECTED_C)? NAN : tBox);
  p.t_hot  = s10_from_float((tHot==DEVICE_DISCONNECTED_C)? NAN : tHot);
  p.t_amb  = s10_from_float((tAmb==DEVICE_DISCONNECTED_C)? NAN : tAmb);
  p.t_cool = s10_from_float((tCold==DEVICE_DISCONNECTED_C)? NAN : tCold);
  p.fridge_on = fridgeOn?1:0;
  p.flags = (sensorsOK?1:0) | (alarmHot? (1<<1):0) | (alarmProbe? (1<<2):0);
  p.crc16 = crc16((uint8_t*)&p, sizeof(p)-2);

  esp_now_send(CONTROLLER_MAC, (uint8_t*)&p, sizeof(p));

  // LCD display (format similar to previous)
  lcd.setCursor(0,1);
  // avoid printing "nan" to LCD: show --.- if disconnected
  auto disp = [](float v)->String{
    if(v==DEVICE_DISCONNECTED_C || !isfinite(v)) return "--.-";
    char buf[8]; sprintf(buf, "%5.1f", v); return String(buf);
  };
  lcd.printf("Box:%sC Hot:%sC  ", disp(tBox).c_str(), disp(tHot).c_str());
  lcd.setCursor(0,2);
  lcd.printf("Cold:%sC  Fridge:%s   ", disp(tCold).c_str(), fridgeOn?"ON ":"OFF");
  lcd.setCursor(0,3);
  uint32_t age = gotAck? (now-lastAckMs):99999;
  lcd.printf("ACK:%s Age:%4lus       ", gotAck?"OK ":"-- ", (unsigned long)(age/1000));

  delay(1000);
}