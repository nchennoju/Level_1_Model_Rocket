#include <Arduino.h>
#include <SPI.h>
#include "RF24.h"

// ============================================================
// RADIO
// ============================================================
RF24 radio(7, 8);                 // CE, CSN
const byte address[6] = "00001";  // Must match TX

// ============================================================
// PACKET UNIONS
// ============================================================
union float_byte {
  float value;
  byte b[4];
};

union short_byte {
  int16_t value;
  byte b[2];
};

// ============================================================
// RECEIVED FIELDS
// ============================================================
union short_byte mode_u;
union float_byte lat_u;
union float_byte lon_u;
union float_byte alt_u;
union short_byte accelMag_u;
union short_byte roll_u;
union short_byte pitch_u;
union short_byte yaw_u;
union short_byte north_u;
union short_byte east_u;
union short_byte yawNav_u;
union short_byte checksum_rx_u;
union short_byte checksum_calc_u;

// ============================================================
// TIMING / STATUS
// ============================================================
unsigned long lastPacketMs = 0;
unsigned long packetCount = 0;
unsigned long badChecksumCount = 0;

// ============================================================
// HELPERS
// ============================================================
bool decodePacket(const byte* buf, size_t len);
void printPacket();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== NRF24 RECEIVER ===");
  Serial.println("Expecting 30-byte telemetry packet");
  Serial.println();

  if (!radio.begin()) {
    Serial.println("radio.begin() failed");
    while (1) {
      delay(100);
    }
  }

  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  Serial.println("Receiver ready.");
}

void loop() {
  if (radio.available()) {
    byte text[30];
    radio.read(&text, sizeof(text));

    if (decodePacket(text, sizeof(text))) {
      lastPacketMs = millis();
      packetCount++;
      printPacket();
    } else {
      badChecksumCount++;
      // Keep this quiet if your GUI expects only JSON.
      // Uncomment if you want checksum debug.
      // Serial.print("Bad checksum. Count=");
      // Serial.println(badChecksumCount);
    }
  }
}

// ============================================================
// PACKET DECODE
// ============================================================
bool decodePacket(const byte* buf, size_t len) {
  if (len != 30) return false;

  int idx = 0;

  // mode
  mode_u.b[0] = buf[idx++];
  mode_u.b[1] = buf[idx++];

  // lat
  for (int i = 0; i < 4; i++) lat_u.b[i] = buf[idx++];

  // lon
  for (int i = 0; i < 4; i++) lon_u.b[i] = buf[idx++];

  // alt
  for (int i = 0; i < 4; i++) alt_u.b[i] = buf[idx++];

  // accelMag
  accelMag_u.b[0] = buf[idx++];
  accelMag_u.b[1] = buf[idx++];

  // roll
  roll_u.b[0] = buf[idx++];
  roll_u.b[1] = buf[idx++];

  // pitch
  pitch_u.b[0] = buf[idx++];
  pitch_u.b[1] = buf[idx++];

  // yaw
  yaw_u.b[0] = buf[idx++];
  yaw_u.b[1] = buf[idx++];

  // north
  north_u.b[0] = buf[idx++];
  north_u.b[1] = buf[idx++];

  // east
  east_u.b[0] = buf[idx++];
  east_u.b[1] = buf[idx++];

  // yawNav
  yawNav_u.b[0] = buf[idx++];
  yawNav_u.b[1] = buf[idx++];

  // checksum rx
  checksum_rx_u.b[0] = buf[idx++];
  checksum_rx_u.b[1] = buf[idx++];

  // checksum calc over first 28 bytes
  checksum_calc_u.value = 0;
  for (int i = 0; i < 28; i++) {
    checksum_calc_u.value += buf[i];
  }

  return (checksum_calc_u.value == checksum_rx_u.value);
}

// ============================================================
// GUI-FRIENDLY JSON PRINT
// ============================================================
void printPacket() {
  // TX sends accel magnitude * 10 in this field.
  // Keeping the original key name "spd_mps" so your GUI keeps working.
  float accel_g = accelMag_u.value / 10.0f;

  String tmp = "{";
  tmp += "\"ts\": " + String(millis() / 1000.0, 3) + ",";
  tmp += "\"alt_m\": " + String(alt_u.value, 2) + ",";
  tmp += "\"spd_mps\": " + String(accel_g, 2) + ",";
  tmp += "\"roll\": " + String((float)roll_u.value, 2) + ",";
  tmp += "\"pitch\": " + String((float)pitch_u.value, 2) + ",";
  tmp += "\"yaw\": " + String((float)yaw_u.value, 2) + ",";
  tmp += "\"lat\": " + String(lat_u.value, 6) + ",";
  tmp += "\"lon\": " + String(lon_u.value, 6) + ",";
  tmp += "\"rssi_dbm\": " + String(0) + "}";

  Serial.println(tmp);
}