// ═══════════════════════════════════════════════════════════════
// IRRIGATION ESP32 — LoRa version (aligned with gateway + Node-RED)
//
// JSON keys sent UP match Node-RED parse_payload exactly:
//   weight_g, soil_pct, input_mL, drainage_mL,
//   water_used_mL, ec_drainage, temp_drainage, pump, drain_pump, id
//
// Commands received DOWN from gateway:
//   {"id":"abdellah","pump":1}          → relay ON
//   {"id":"abdellah","pump":0}          → relay OFF
//   {"id":"abdellah","drain_pump":1}    → drain relay ON
//   {"id":"abdellah","drain_pump":0}    → drain relay OFF
// ═══════════════════════════════════════════════════════════════

#include <SPI.h>
#include <LoRa.h>
#include "HX711.h"
#include "DFRobot_ECPRO.h"

// ─────────────────────────────
// DEVICE ID
// ─────────────────────────────
const String DEVICE_ID = "abdellah";

// ─────────────────────────────
// LORA
// ─────────────────────────────
#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  2
#define LORA_BAND  868E6

// ─────────────────────────────
// RELAYS
// LOW  = relay energised (pump ON)
// HIGH = relay off       (pump OFF)
// ─────────────────────────────
#define RELAY_PIN  25   // main irrigation pump
#define RELAY2_PIN 4   // drainage pump

bool pumpState      = false;
bool drainPumpState = false;

// ─────────────────────────────
// HX711
// ─────────────────────────────
#define DOUT 26
#define CLK  27
HX711 scale;
const float CALIBRATION_FACTOR = 480245.0;

// ─────────────────────────────
// SOIL
// ─────────────────────────────fd
#define SOIL_PIN 32
#define SOIL_DRY 4095
#define SOIL_WET 1200

// ─────────────────────────────
// FLOW SENSORS (interrupt-based)
// ─────────────────────────────
#define FLOW1_PIN 21
#define FLOW2_PIN 22
#define FLOW_CAL  450.0   // pulses per litre

volatile long inputPulses = 0;
volatile long drainPulses = 0;

// Accumulated totals in mL — persist across sends
float input_mL    = 0.0;
float drainage_mL = 0.0;

void IRAM_ATTR flow1ISR() { inputPulses++; }
void IRAM_ATTR flow2ISR() { drainPulses++; }

// ─────────────────────────────
// EC + TEMP
// ─────────────────────────────
#define EC_PIN 34
#define TE_PIN 35

DFRobot_ECPRO        ec;
DFRobot_ECPRO_PT1000 ecpt;

float ec_drainage   = 0.0;
float temp_drainage = 0.0;

// ─────────────────────────────
// TIMING
// ─────────────────────────────
unsigned long lastSend    = 0;
const unsigned long SEND_INTERVAL_MS = 30000UL;  // 30 seconds

// ─────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n===== ESP32 LoRa Irrigation Node =====");
  Serial.println("ID: " + DEVICE_ID);

  // ── Relays — HIGH = OFF at startup ──────────
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY_PIN,  HIGH);
  digitalWrite(RELAY2_PIN, LOW);

  // ── HX711 ───────────────────────────────────
  scale.begin(DOUT, CLK);
  scale.set_scale(CALIBRATION_FACTOR);
  scale.tare();
  Serial.println("✅ Scale ready");

  // ── EC sensor ───────────────────────────────
  ec.setCalibration(0.961);

  // ── Soil ────────────────────────────────────
  pinMode(SOIL_PIN, INPUT);

  // ── Flow interrupts ─────────────────────────
  pinMode(FLOW1_PIN, INPUT_PULLUP);
  pinMode(FLOW2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW1_PIN), flow1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW2_PIN), flow2ISR, RISING);

  // ── LoRa ────────────────────────────────────
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("❌ LoRa init failed — halting");
    while (true);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);
  Serial.println("✅ LoRa ready at 868 MHz");
}

// ─────────────────────────────────────────────
// RECEIVE LORA COMMANDS
// Called every loop iteration to check for
// incoming packets from the gateway
// ─────────────────────────────────────────────
void receiveLoRa() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;

  String msg = "";
  while (LoRa.available()) {
    msg += (char)LoRa.read();
  }
  msg.trim();
  Serial.println("\n📥 LoRa RX: " + msg);

  // ── filter by device id ──────────────────────
  if (msg.indexOf("\"id\":\"" + DEVICE_ID + "\"") == -1) {
    Serial.println("⏩ Ignored — not for " + DEVICE_ID);
    return;
  }

  // ── main pump ────────────────────────────────
  // Node-RED/gateway sends pump:1 (ON) or pump:0 (OFF)
  if (msg.indexOf("\"pump\":1") >= 0) {
    pumpState = true;
    digitalWrite(RELAY_PIN, LOW);   // LOW = energise relay = pump ON
    Serial.println("✅ Main pump ON");
  } else if (msg.indexOf("\"pump\":0") >= 0) {
    pumpState = false;
    digitalWrite(RELAY_PIN, HIGH);
    Serial.println("🔴 Main pump OFF");
  }

  // ── drain pump ───────────────────────────────
  if (msg.indexOf("\"drain_pump\":1") >= 0) {
    drainPumpState = true;
    digitalWrite(RELAY2_PIN, HIGH);
    Serial.println("✅ Drain pump ON");
  } else if (msg.indexOf("\"drain_pump\":0") >= 0) {
    drainPumpState = false;
    digitalWrite(RELAY2_PIN, LOW);
    Serial.println("🔴 Drain pump OFF");
  }

  // ── RSSI for signal quality log ──────────────
  Serial.println("📶 RSSI: " + String(LoRa.packetRssi()) + " dBm");
}

// ─────────────────────────────────────────────
// READ SENSORS AND SEND VIA LORA
// ─────────────────────────────────────────────
void sendData() {

  // ── Weight ───────────────────────────────────
  float weight_g = scale.get_units(5) * 1000.0;
  if (weight_g < 0) weight_g = 0;

  // ── Soil moisture ────────────────────────────
  int soilRaw = analogRead(SOIL_PIN);
  int soil_pct = constrain(
    map(soilRaw, SOIL_DRY, SOIL_WET, 0, 100),
    0, 100
  );

  // ── Flow accumulation ────────────────────────
  // Snapshot and reset interrupt counters atomically
  noInterrupts();
  long inPulses = inputPulses;
  long drPulses = drainPulses;
  inputPulses   = 0;
  drainPulses   = 0;
  interrupts();

  // Convert pulses → mL and accumulate
  input_mL    += (inPulses / FLOW_CAL) * 1000.0;
  drainage_mL += (drPulses / FLOW_CAL) * 1000.0;

  float water_used_mL = input_mL - drainage_mL;
  if (water_used_mL < 0) water_used_mL = 0;

  // ── EC + Temperature (DFRobot library) ───────
  // Only meaningful when drainage is flowing
  if (drainage_mL > 0) {
    delay(200); // let ADC settle

    // Temperature from PT1000
    uint16_t te_mv = (uint16_t)(analogRead(TE_PIN) * 3300UL / 4095);
    temp_drainage = ecpt.convVoltagetoTemperature_C(te_mv / 1000.0);

    // EC with temperature compensation
    uint16_t ec_mv = (uint16_t)(analogRead(EC_PIN) * 3300UL / 4095);
    ec_drainage    = ec.getEC_us_cm(ec_mv, temp_drainage);
  } else {
    // No drainage yet — report 0
    ec_drainage   = 0.0;
    temp_drainage = 0.0;
  }

  // ── Build JSON — keys match Node-RED exactly ──
  String json = "{";
  json += "\"id\":\""          + DEVICE_ID                        + "\",";
  json += "\"weight_g\":"      + String((int)round(weight_g))     + ",";
  json += "\"soil_pct\":"      + String(soil_pct)                 + ",";
  json += "\"input_mL\":"      + String((int)round(input_mL))     + ",";
  json += "\"drainage_mL\":"   + String((int)round(drainage_mL))  + ",";
  json += "\"water_used_mL\":" + String((int)round(water_used_mL))+ ",";
  json += "\"ec_drainage\":"   + String((int)round(ec_drainage))  + ",";
  json += "\"temp_drainage\":" + String(temp_drainage, 1)         + ",";
  json += "\"pump\":\""        + String(pumpState      ? "ON":"OFF") + "\",";
  json += "\"drain_pump\":\""  + String(drainPumpState ? "ON":"OFF") + "\"";
  json += "}";

  // ── Send via LoRa ─────────────────────────────
  LoRa.beginPacket();
  LoRa.println(json);
  LoRa.endPacket();

  Serial.println("📤 LoRa TX: " + json);
  Serial.println("   weight:" + String((int)weight_g) +
                 "g soil:" + String(soil_pct) +
                 "% in:" + String((int)input_mL) +
                 "mL drain:" + String((int)drainage_mL) +
                 "mL ec:" + String((int)ec_drainage) +
                 "uS/cm temp:" + String(temp_drainage, 1) + "°C");
}

// ─────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────
void loop() {
  // Always listen for incoming commands
  receiveLoRa();

  // Send sensor data every SEND_INTERVAL_MS
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    sendData();
    lastSend = millis();
  }

  delay(50);
}
