// ═══════════════════════════════════════════════════════════════
// IRRIGATION ESP32 — LoRa version (aligned with gateway + Node-RED)
// FIX: receiveLoRa() called throughout sendData() to prevent
//      missed commands during blocking sensor reads.
// NEW: Deep sleep during "repos" (pump OFF) — wakes every 5 min
//      to send telemetry. While pump is ON, deep sleep is
//      disabled and the device stays fully awake/responsive
//      (30s telemetry + continuous LoRa listening), exactly
//      like before.
// ═══════════════════════════════════════════════════════════════

#include <SPI.h>
#include <LoRa.h>
#include "HX711.h"
#include "DFRobot_ECPRO.h"
#include "esp_sleep.h"

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
// ─────────────────────────────
#define RELAY_PIN  25
#define RELAY2_PIN 4

// NOTE: RTC_DATA_ATTR keeps these values alive across deep sleep
// (deep sleep wipes normal RAM but preserves RTC memory).
RTC_DATA_ATTR bool  pumpState      = false;
RTC_DATA_ATTR bool  drainPumpState = false;
RTC_DATA_ATTR float input_mL       = 0.0;
RTC_DATA_ATTR float drainage_mL    = 0.0;
RTC_DATA_ATTR int   bootCount      = 0;

// ─────────────────────────────
// HX711
// ─────────────────────────────
#define DOUT 26
#define CLK  27
HX711 scale;
const float CALIBRATION_FACTOR = 480245.0;

// ─────────────────────────────
// SOIL SENSOR
// ─────────────────────────────
#define SOIL_PIN 33
int SOIL_DRY = 4095;
int SOIL_WET = 1200;

// ─────────────────────────────
// FLOW SENSORS
// ─────────────────────────────
#define FLOW1_PIN 13
#define FLOW2_PIN 12
#define FLOW_CAL  450.0

volatile long inputPulses = 0;
volatile long drainPulses = 0;

void IRAM_ATTR flow1ISR() { inputPulses++; }
void IRAM_ATTR flow2ISR() { drainPulses++; }

// ─────────────────────────────
// EC + TEMP
// ─────────────────────────────
#define EC_PIN 34
#define TE_PIN 35

DFRobot_ECPRO ec;
DFRobot_ECPRO_PT1000 ecpt;

float ec_drainage   = 0.0;
float temp_drainage = 0.0;

// ─────────────────────────────
// TIMING (active / irrigation mode)
// ─────────────────────────────
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 30000UL; // 30s while pump is ON

// ─────────────────────────────
// DEEP SLEEP (repos / resting mode)
// ─────────────────────────────
#define uS_TO_S_FACTOR 1000000ULL
const uint64_t SLEEP_MINUTES     = 5;                                 // send data every 5 min while resting
const uint64_t SLEEP_INTERVAL_US = SLEEP_MINUTES * 60ULL * uS_TO_S_FACTOR;
const unsigned long LISTEN_WINDOW_MS = 3000UL; // brief LoRa listen right after wake-up

// ═══════════════════════════════════════════════════════════════
// PH
// ═══════════════════════════════════════════════════════════════
const int    PH_PIN    = 32;
const float  slope     = -3.7;
const float  intercept = 16.15;

float readPH() {
  const int samples = 20;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(PH_PIN);
    delay(2);
  }
  float avg     = sum / (float)samples;
  float voltage = avg * 3.3 / 4095.0;
  return slope * voltage + intercept;
}

// ═══════════════════════════════════════════════════════════════
// SOIL
// ═══════════════════════════════════════════════════════════════
int readSoil() {
  const int samples = 20;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(SOIL_PIN);
    delay(2);
  }
  float avg = sum / (float)samples;
  int pct = map((int)avg, SOIL_DRY, SOIL_WET, 0, 100);
  return constrain(pct, 0, 100);
}

// ═══════════════════════════════════════════════════════════════
// RECEIVE LORA
// ═══════════════════════════════════════════════════════════════
void receiveLoRa() {
  int packetSize = LoRa.parsePacket();
  if (!packetSize) return;

  String msg = "";
  while (LoRa.available()) {
    msg += (char)LoRa.read();
  }

  Serial.println("══════════════════════");
  Serial.println("📡 LoRa message received:");
  Serial.println(msg);
  Serial.println("══════════════════════");

  if (msg.indexOf("\"id\":\"abdellah\"") == -1) return;

  // ── Main pump (ACTIVE LOW relay) ──
  if (msg.indexOf("\"pump\":1") >= 0) {
    pumpState = true;
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("✅ pump ON");
  }
  if (msg.indexOf("\"pump\":0") >= 0) {
    pumpState = false;
    digitalWrite(RELAY_PIN, HIGH);
    Serial.println("✅ pump OFF");
  }

  // ── Drain pump (ACTIVE HIGH relay) ──
  if (msg.indexOf("\"drain_pump\":1") >= 0) {
    drainPumpState = true;
    digitalWrite(RELAY2_PIN, HIGH);
    Serial.println("✅ drain_pump ON");
  }
  if (msg.indexOf("\"drain_pump\":0") >= 0) {
    drainPumpState = false;
    digitalWrite(RELAY2_PIN, LOW);
    Serial.println("✅ drain_pump OFF");
  }
}

// ═══════════════════════════════════════════════════════════════
// DEEP SLEEP HELPERS
// ═══════════════════════════════════════════════════════════════
void printWakeupReason() {
  esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
  switch (reason) {
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println(" Woke up from deep sleep (timer, repos mode)");
      break;
    default:
      Serial.println(" Cold boot / power-on");
      break;
  }
}

// Only called when pump AND drain_pump are OFF (repos time).
// Never returns — the chip restarts from setup() on wake-up.
void goToDeepSleep() {
  Serial.println("💤 Repos mode: pump OFF — entering deep sleep for 5 minutes");
  Serial.flush();
  esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US);
  esp_deep_sleep_start();
}

// ═══════════════════════════════════════════════════════════════
// SEND DATA
// ═══════════════════════════════════════════════════════════════
void sendData() {

  // ── 1. Weight (HX711 ~500ms blocking) ──
  receiveLoRa();
  float weight_g = 0;
  if (scale.is_ready()) {
    weight_g = scale.get_units(5) * 1000.0;
    if (weight_g < 0) weight_g = 0;
  } else {
    Serial.println(" HX711 not ready — skipping weight");
  }

  // ── 2. Soil (~40ms blocking) ──
  receiveLoRa();
  int soil_pct = readSoil();

  // ── 3. Flow pulse snapshot (non-blocking) ──
  receiveLoRa();
  noInterrupts();
  long inP = inputPulses;
  long drP = drainPulses;
  inputPulses  = 0;
  drainPulses  = 0;
  interrupts();

  input_mL    += (inP / FLOW_CAL) * 1000.0;
  drainage_mL += (drP / FLOW_CAL) * 1000.0;
  float water_used = input_mL - drainage_mL;
  if (water_used < 0) water_used = 0;

  // ── 4. EC (~40ms blocking, only if drainage detected) ──
  receiveLoRa();
  if (drainage_mL > 0) {
    const int samples = 20;
    long sum = 0;
    for (int i = 0; i < samples; i++) {
      sum += analogRead(EC_PIN);
      delay(2);
    }
    float avg      = sum / (float)samples;
    float voltage  = avg * 3.3 / 4095.0;
    temp_drainage  = 25.0;
    ec_drainage    = ec.getEC_us_cm(voltage * 1000.0, temp_drainage);
    ec_drainage   *= 0.999;
  } else {
    ec_drainage   = 0;
    temp_drainage = 0;
  }

  // ── 5. pH (~40ms blocking) ──
  receiveLoRa();
  float ph_value = readPH();

  // ── 6. Build JSON & transmit ──
  receiveLoRa();

  String json = "{";
  json += "\"id\":\""          + DEVICE_ID                              + "\",";
  json += "\"weight_g\":"      + String((int)round(weight_g))           + ",";
  json += "\"soil_pct\":"      + String(soil_pct)                       + ",";
  json += "\"ph\":"            + String(ph_value, 2)                    + ",";
  json += "\"input_mL\":"      + String((int)round(input_mL))           + ",";
  json += "\"drainage_mL\":"   + String((int)round(drainage_mL))        + ",";
  json += "\"water_used_mL\":" + String((int)round(water_used))         + ",";
  json += "\"ec_drainage\":"   + String((int)round(ec_drainage))        + ",";
  json += "\"temp_drainage\":" + String(temp_drainage, 1)               + ",";
  json += "\"pump\":\""        + String(pumpState      ? "ON" : "OFF")  + "\",";
  json += "\"drain_pump\":\""  + String(drainPumpState ? "ON" : "OFF")  + "\"";
  json += "}";

  LoRa.beginPacket();
  LoRa.println(json);
  LoRa.endPacket();

  Serial.println(json);
}

// ═══════════════════════════════════════════════════════════════
// SETUP
// Runs on every cold boot AND every deep-sleep wake-up.
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);

  bootCount++;
  Serial.println("Boot count: " + String(bootCount));
  printWakeupReason();

  pinMode(RELAY_PIN,  OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  // Restore relay outputs to match the persisted state
  // (deep sleep resets GPIOs, so we must re-apply them).
  digitalWrite(RELAY_PIN,  pumpState      ? LOW  : HIGH); // active low
  digitalWrite(RELAY2_PIN, drainPumpState ? HIGH : LOW);  // active high

  scale.begin(DOUT, CLK);
  scale.set_scale(CALIBRATION_FACTOR);
  scale.tare();

  pinMode(SOIL_PIN,  INPUT);
  pinMode(FLOW1_PIN, INPUT_PULLUP);
  pinMode(FLOW2_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FLOW1_PIN), flow1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW2_PIN), flow2ISR, RISING);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("LoRa init failed!");
    while (true);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);

  Serial.println("ESP32 ready.");

  // ── Short listen window right after wake-up ──
  // Catches a command that might arrive just as we wake, since
  // the radio was completely off during deep sleep.
  unsigned long listenStart = millis();
  while (millis() - listenStart < LISTEN_WINDOW_MS) {
    receiveLoRa();
    delay(10);
  }

  // Send one telemetry packet immediately (also true for cold boot).
  lastSend = millis();
  sendData();

  // ── Decide: stay awake (irrigation active) or go back to sleep (repos) ──
  if (!pumpState && !drainPumpState) {
    goToDeepSleep(); // never returns — chip restarts on next wake
  }
  // else: pump is ON -> fall through into loop() and stay fully awake
}

// ═══════════════════════════════════════════════════════════════
// LOOP
// Only reached while pump (or drain_pump) is ON — i.e. irrigation
// is active. Behaves exactly like the original always-on version:
// continuous LoRa listening + telemetry every 30s.
// ═══════════════════════════════════════════════════════════════
void loop() {
  receiveLoRa();

  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();   // set BEFORE sendData() so a slow
    sendData();            // read never causes double-fire

    // Pump may have just been turned OFF during sendData()'s
    // receiveLoRa() calls -> switch to repos/deep-sleep mode.
    if (!pumpState && !drainPumpState) {
      goToDeepSleep();
    }
  }

  delay(10);
}
