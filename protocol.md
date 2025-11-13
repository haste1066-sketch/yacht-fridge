# ESP-NOW Packet

// Sender → Controller
struct TxPacket {
  uint32_t seq;         // increments every message
  uint32_t millis32;    // sender uptime low 32 bits
  int16_t  t_box_c10;   // °C * 10
  int16_t  t_hot_c10;
  int16_t  t_amb_c10;
  int16_t  t_coolant_c10; // -32768 if N/A
  uint8_t  fridge_on;   // 1 ON, 0 OFF
  uint8_t  flags;       // bit0: sensors_ok, bit1: alarm_hot, bit2: alarm_probe
  uint16_t crc16;       // CRC-16/IBM of previous bytes
} __attribute__((packed));

// Controller → Sender (ACK-lite)
struct RxAck {
  uint32_t ack_seq;     // echoes seq
  int8_t   rssi;        // last RSSI
  uint8_t  outputs;     // bit0 TEC, bit1 fan_hot, bit2 fan_box, bit3 pump1, bit4 pump2
  uint8_t  health;      // bit0 fail_safe, bit1 hot_trip, bit2 undervolt
  uint16_t crc16;
} __attribute__((packed));
