#include <SPI.h>
#include <LoRa.h>

#define SS    10
#define RST    9
#define DIO0   2

String serialBuffer = "";

bool isAllowedID(String data) {
  data.toLowerCase();

  return (data.indexOf("\"id\":\"abdelghani\"") != -1 ||
          data.indexOf("\"id\":\"abdellah\"") != -1 || data.indexOf("\"id\":\"jihad\"") != -1);
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(868E6)) {
    while (1); // Halt on failure
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);

  serialBuffer.reserve(256);
}

void loop() {

  // ═══════════════════════════════════════════════
  // 📥 RECEIVE: LoRa → Raspberry Pi (FILTERED)
  // ═══════════════════════════════════════════════
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    String received = "";

    while (LoRa.available()) {
      received += (char)LoRa.read();
    }

    received.trim();

    // فقط JSON + ID autorisé
    if (received.startsWith("{") && received.endsWith("}")) {

      if (isAllowedID(received)) {
        Serial.println(received);  // send to Raspberry Pi only allowed IDs
      }
    }
  }

  // ═══════════════════════════════════════════════
  // 📤 TRANSMIT: Raspberry Pi → LoRa
  // ═══════════════════════════════════════════════
  if (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {

      if (serialBuffer.length() > 0) {
        serialBuffer.trim();

        if (serialBuffer.startsWith("{") && serialBuffer.endsWith("}")) {
          delay(10);

          LoRa.beginPacket();
          LoRa.print(serialBuffer);
          LoRa.endPacket();

          delay(50);
        }

        serialBuffer = "";
      }

    } else {
      serialBuffer += c;

      if (serialBuffer.length() > 250) {
        serialBuffer = "";
      }
    }
  }

  delay(5);
}