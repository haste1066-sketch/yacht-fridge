#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_now.h>
#include <WiFi.h>

// ------- CONFIG SNIPPET (inline for demo; mirror values from config.yaml) -------
static const uint8_t CONTROLLER_MAC[6] = {0x24,0x0A,0xC4,0xAA,0xBB,0xCC};
const int LCD_ADDR=0x27, LCD_COLS=20, LCD_ROWS=4, I2C_SDA=21, I2C_SCL=22;
const int PIN_T_BOX=34, PIN_T_HOT=35, PIN_T_AMB=32, PIN_T_COOL=33;
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

// Simple NTC conversion (edit to match sensors)
float analogToCelsius(int pin){
  int raw = analogRead(pin);
  if(raw<=0) return NAN;
  const float Vref=3.3, ADCmax=4095.0;
  float v = (raw/ADCmax)*Vref;
  const float Rseries=10000.0, Beta=3950.0, R0=10000.0, T0=298.15;
  float R = (v>0.0001)? (Rseries * (Vref/v - 1.0)) : 1e9;
  float invT = 1.0/T0 + (1.0/Beta)*log(R/R0);
  return (1.0/invT) - 273.15;
}

bool fridgeOn=false;
uint32_t lastToggle=0;
uint32_t seq=0;

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

void setup(){
  Serial.begin(115200);
  analogReadResolution(12);
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init(); lcd.backlight();
  lcd.clear(); lcd.setCursor(0,0); lcd.print("Yacht Fridge: SENDER");

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CH, WIFI_SECOND_CHAN_NONE);
  if(esp_now_init()!=ESP_OK){ lcd.setCursor(0,1); lcd.print("ESP-NOW init fail"); return; }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  esp_now_peer_info_t peer{}; memcpy(peer.peer_addr, CONTROLLER_MAC, 6);
  peer.channel=ESPNOW_CH; peer.encrypt=false;
  esp_now_add_peer(&peer);
}

void loop(){
  // Read sensors
  float tBox=analogToCelsius(PIN_T_BOX);
  float tHot=analogToCelsius(PIN_T_HOT);
  float tAmb=analogToCelsius(PIN_T_AMB);
  float tCool=analogToCelsius(PIN_T_COOL); // may be NAN if unused

  bool sensorsOK = isfinite(tBox) && isfinite(tHot);
  bool alarmHot  = sensorsOK && (tHot > 65.0);
  bool alarmProbe= !sensorsOK;

  // Hysteresis & anti short-cycle
  uint32_t now=millis();
  if(!fridgeOn){
    if(tBox > (SETPOINT+ON_DELTA) && (now-lastToggle)>=MIN_OFF_MS) { fridgeOn=true; lastToggle=now; }
  } else {
    if(tBox < (SETPOINT-OFF_DELTA) && (now-lastToggle)>=MIN_ON_MS) { fridgeOn=false; lastToggle=now; }
  }

  // Build & send packet
  TxPacket p{};
  p.seq = ++seq;
  p.millis32 = now;
  auto s10 = [](float c)->int16_t{ return isfinite(c)? (int16_t)round(c*10.0): (int16_t)-32768; };
  p.t_box = s10(tBox); p.t_hot = s10(tHot); p.t_amb = s10(tAmb); p.t_cool = s10(tCool);
  p.fridge_on = fridgeOn?1:0;
  p.flags = (sensorsOK?1:0) | (alarmHot? (1<<1):0) | (alarmProbe? (1<<2):0);
  p.crc16 = crc16((uint8_t*)&p, sizeof(p)-2);

  esp_now_send(CONTROLLER_MAC, (uint8_t*)&p, sizeof(p));

  // LCD
  lcd.setCursor(0,1);
  lcd.printf("Box:%5.1fC Hot:%5.1fC  ", tBox, tHot);
  lcd.setCursor(0,2);
  lcd.printf("Amb:%5.1fC  Fridge:%s   ", tAmb, fridgeOn?"ON ":"OFF");
  lcd.setCursor(0,3);
  uint32_t age = gotAck? (now-lastAckMs):99999;
  lcd.printf("ACK:%s Age:%4lus       ", gotAck?"OK ":"-- ", (unsigned long)(age/1000));

  delay(1000);
}
