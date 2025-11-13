#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_now.h>
#include <WiFi.h>

static const uint8_t SENDER_MAC[6] = {0x24,0x0A,0xC4,0xDD,0xEE,0xFF};
const int LCD_ADDR=0x27, LCD_COLS=20, LCD_ROWS=4, I2C_SDA=21, I2C_SCL=22;
const int PIN_TEC=25, PIN_FAN_HOT=26, PIN_FAN_BOX=27, PIN_PUMP1=14, PIN_PUMP2=12;
const bool SEAWATER_PRESENT=false;
const int PWM_FREQ=25000, PWM_RES=8;
const int CH_FAN_HOT=0, CH_FAN_BOX=1;
const uint32_t PACKET_TIMEOUT_MS=4000, POSTRUN_PUMP_MS=20000, DEFROST_IDLE_DUTY=20;
const float HOT_MAX_C=65.0;
const int ESPNOW_CH=6;

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

TxPacket lastPkt{};
bool havePkt=false;
uint32_t lastPktMs=0;
bool tec=false, pump1=false, pump2=false;
int fanHotDuty=0, fanBoxDuty=0;
uint32_t tecOffTime=0;

uint16_t crc16(const uint8_t* d, size_t n){
  uint16_t c=0xFFFF;
  for(size_t i=0;i<n;i++){ c^=d[i]; for(int j=0;j<8;j++) c=(c&1)?(c>>1)^0xA001:(c>>1); }
  return c;
}

void applyOutputs(bool fridge_on, float tHot){
  uint32_t now=millis();
  bool timeout = !havePkt || (now - lastPktMs > PACKET_TIMEOUT_MS);
  bool hotTrip = (isfinite(tHot) && tHot > HOT_MAX_C);

  if(timeout){
    tec=false; pump1=false; pump2=false;
    fanHotDuty=DEFROST_IDLE_DUTY; fanBoxDuty=DEFROST_IDLE_DUTY;
  } else if(hotTrip){
    tec=false; pump1=true; pump2=SEAWATER_PRESENT; // pull heat away
    fanHotDuty=255; fanBoxDuty=DEFROST_IDLE_DUTY;
  } else {
    tec = fridge_on;
    if(tec){ pump1=true; pump2=SEAWATER_PRESENT; fanHotDuty=255; fanBoxDuty=200; }
    else {
      // Post-run pumps to remove heat soak
      if(tecOffTime==0) tecOffTime=now;
      bool inPost = (now - tecOffTime) < POSTRUN_PUMP_MS;
      pump1 = inPost; pump2 = SEAWATER_PRESENT && inPost;
      fanHotDuty = inPost? 180 : DEFROST_IDLE_DUTY;
      fanBoxDuty = DEFROST_IDLE_DUTY;
    }
    if(tec) tecOffTime=0;
  }

  // Write hardware
  digitalWrite(PIN_TEC, tec?HIGH:LOW);
  digitalWrite(PIN_PUMP1, pump1?HIGH:LOW);
  digitalWrite(PIN_PUMP2, pump2?HIGH:LOW);
  ledcWrite(CH_FAN_HOT, fanHotDuty);
  ledcWrite(CH_FAN_BOX, fanBoxDuty);
}

void onDataRecv(const uint8_t* mac, const uint8_t* data, int len){
  if(len==(int)sizeof(TxPacket)){
    TxPacket pkt; memcpy(&pkt, data, len);
    uint16_t c = crc16((uint8_t*)&pkt, sizeof(pkt)-2);
    if(c==pkt.crc16){
      lastPkt=pkt; havePkt=true; lastPktMs=millis();
      // Immediately respond with ACK
      struct RxAck {
        uint32_t ack_seq; int8_t rssi; uint8_t outputs, health; uint16_t crc16;
      } ack{};
      ack.ack_seq = pkt.seq;
      // RSSI not directly available here; leave 0
      ack.outputs = (tec?1:0) | (fanHotDuty>0? (1<<1):0) | (fanBoxDuty>0? (1<<2):0)
                  | (pump1? (1<<3):0) | (pump2? (1<<4):0);
      bool timeout = (millis()-lastPktMs) > PACKET_TIMEOUT_MS;
      bool hotTrip = (lastPkt.t_hot!= (int16_t)-32768 && (lastPkt.t_hot/10.0)>HOT_MAX_C);
      ack.health = (timeout?1:0) | (hotTrip? (1<<1):0);
      ack.crc16 = crc16((uint8_t*)&ack, sizeof(ack)-2);
      esp_now_send(mac, (uint8_t*)&ack, sizeof(ack));
    }
  }
}

void setup(){
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init(); lcd.backlight();
  pinMode(PIN_TEC, OUTPUT); pinMode(PIN_PUMP1, OUTPUT); pinMode(PIN_PUMP2, OUTPUT);
  ledcSetup(CH_FAN_HOT, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_FAN_HOT, CH_FAN_HOT);
  ledcSetup(CH_FAN_BOX, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_FAN_BOX, CH_FAN_BOX);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CH, WIFI_SECOND_CHAN_NONE);
  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);
  esp_now_peer_info_t peer{}; memcpy(peer.peer_addr, SENDER_MAC, 6);
  peer.channel=ESPNOW_CH; peer.encrypt=false; esp_now_add_peer(&peer);

  lcd.setCursor(0,0); lcd.print("Controller Ready");
}

void loop(){
  float tHot = (lastPkt.t_hot==(int16_t)-32768)? NAN : (lastPkt.t_hot/10.0f);
  bool fridgeReq = havePkt && lastPkt.fridge_on;

  applyOutputs(fridgeReq, tHot);

  // LCD
  lcd.setCursor(0,1);
  float tBox = (lastPkt.t_box==(int16_t)-32768)? NAN : (lastPkt.t_box/10.0f);
  lcd.printf("Box:%5.1fC Hot:%5.1fC   ", tBox, tHot);
  lcd.setCursor(0,2);
  lcd.printf("TEC:%s FH:%3d FB:%3d    ", tec?"ON ":"OFF", fanHotDuty, fanBoxDuty);
  lcd.setCursor(0,3);
  uint32_t age = havePkt? (millis()-lastPktMs):99999;
  lcd.printf("P1:%s P2:%s Age:%4lus", pump1?"ON ":"OFF", pump2?"ON ":"OFF", (unsigned long)(age/1000));

  delay(500);
}
