/*
 * ESP32 Dual ADC Simultaneous Sampling with Phase Detection
 * Captures one full cycle (20ms @ 50Hz) with timestamps
 * Calculates phase difference to determine leading/lagging
 * Computes power factor for automatic PFC correction
 *
 * Features:
 *   - I2C LCD display: power factor, phase status, voltage, active capacitors
 *   - Blynk IoT app: streams PF, phase angle, and voltage in real-time
 *
 * IMPORTANT: Both ADC channels use ADC1 so that WiFi/Blynk can run without conflict.
 *   ADC2 (GPIO 4) shares the ESP32 WiFi radio and must NOT be used when WiFi is active.
 *
 * Confirmed hardware (customer circuit):
 *   - Voltage sense: 12V AC → 1:9 resistor divider → biased to 1.65V mid-rail → GPIO 36
 *   - Current sense: CT coil → 330Ω burden resistor → biased to mid-rail → GPIO 34
 *   - Load: AC fan (inductive)
 *   - Relay 1: 5µF capacitor bank
 *   - Relay 2: 12µF capacitor bank
 *
 * Pin summary:
 *   GPIO 36  - Voltage sense (ADC1_CH0)
 *   GPIO 34  - Current sense (ADC1_CH6)  ← physical wire moved from GPIO 4
 *   GPIO 26  - Relay 1 (5µF capacitor)
 *   GPIO 25  - Relay 2 (12µF capacitor)
 *   GPIO 2   - Status LED
 *   GPIO 21  - I2C SDA (LCD)
 *   GPIO 22  - I2C SCL (LCD)
 *
 * Libraries required (install via Arduino Library Manager):
 *   - LiquidCrystal_I2C  (by Frank de Brabander)
 *   - Blynk              (by Volodymyr Shymanskyy)
 *
 * CALIBRATION STEPS (do once, then leave):
 *   1. Set RAW_DEBUG_MODE 1 below, upload, open Serial Monitor at 115200
 *      With NO mains connected both channels should idle near 2048.
 *      If they do not, measure the mid-rail pin voltage with a multimeter
 *      then calculate: ADC_OFFSET = (V_mid / 3.3) x 4096  and update below.
 *   2. Connect mains + fan. Voltage RMS on serial should match your mains voltage.
 *      If it does not: VOLT_RATIO = actual_mains_voltage / displayed_voltage * current_VOLT_RATIO
 *   3. Current RMS should match a clamp meter reading.
 *      If it does not: CT_RATIO = actual_amps / displayed_amps * current_CT_RATIO
 *   4. Set RAW_DEBUG_MODE 0 and re-upload for clean operation.
 */

#define BLYNK_TEMPLATE_ID   "YOUR_TEMPLATE_ID"    // ← replace with your Blynk template ID
#define BLYNK_TEMPLATE_NAME "PFC Controller"
#define BLYNK_AUTH_TOKEN    "YOUR_AUTH_TOKEN"      // ← replace with your Blynk auth token

#include <Arduino.h>
#include <cmath>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// ============= WIFI CREDENTIALS =============
const char* WIFI_SSID = "YOUR_WIFI_SSID";         // ← replace with your WiFi network name
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";      // ← replace with your WiFi password

// ============= BLYNK VIRTUAL PIN MAPPING =============
// Configure matching widgets in the Blynk app (Gauge or Value Display):
#define VPIN_VOLTAGE     V0   // Voltage RMS (V)
#define VPIN_POWER_FACTOR V1  // Power Factor (0.00 – 1.00)
#define VPIN_PHASE_ANGLE V2   // Phase Angle (degrees)
#define VPIN_PHASE_TYPE  V3   // Phase type: 0 = IN_PHASE, 1 = LAGGING, 2 = LEADING
#define VPIN_RELAY_5UF   V4   // Relay 1 state (0/1)
#define VPIN_RELAY_12UF  V5   // Relay 2 state (0/1)

// ============= PIN CONFIGURATION =============
#define VOLT_PIN_ADC1  36   // ADC1_CH0 (GPIO 36) - Voltage
#define CURR_PIN_ADC1  34   // ADC1_CH6 (GPIO 34) - Current (ADC1 only — safe with WiFi)
#define RELAY_5UF_PIN  26   // Relay 1 (5µF)
#define RELAY_12UF_PIN 25   // Relay 2 (12µF)
#define LED_PIN         2   // Status LED
// LCD uses default I2C bus: SDA = GPIO 21, SCL = GPIO 22

// ============= I2C LCD =============
// Common I2C address is 0x27; try 0x3F if display is blank
#define LCD_I2C_ADDR 0x27
#define LCD_COLS     16
#define LCD_ROWS      2
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

// ============= SAMPLING CONFIGURATION =============
#define AC_FREQUENCY 50                                           // 50Hz or 60Hz
#define FULL_CYCLE_TIME_US (1000000UL / AC_FREQUENCY)             // 20000µs for 50Hz
#define SAMPLE_INTERVAL_US 100UL                                  // Sample every 100µs
#define TOTAL_SAMPLES (FULL_CYCLE_TIME_US / SAMPLE_INTERVAL_US)   // 200 samples per full cycle

// Approximate ADC read time per channel (µs). Measured ~5µs on ESP32 @ 80MHz.
// Two reads per iteration, so subtract 2x from the delay to keep intervals accurate.
#define ADC_READ_TIME_US 5UL
#define ADJUSTED_DELAY_US (SAMPLE_INTERVAL_US - 2 * ADC_READ_TIME_US)  // 90µs

// ============= SENSOR CALIBRATION =============
// --- HOW TO FIND YOUR VALUES ---
//
// VOLT_RATIO:
//   This is the total scaling factor from mains voltage down to the ADC pin.
//   It is the product of the transformer ratio AND the resistor divider ratio.
//
//   Your hardware chain:
//     Mains (225V AC)
//       → Transformer (225V : 12.5V)   ratio = 225 / 12.5 = 18
//       → Resistor divider (9kΩ top, 1kΩ bottom, ADC reads across 1kΩ)
//                                       ratio = (9k + 1k) / 1k = 10
//       → ADC pin
//
//   VOLT_RATIO = transformer_ratio × divider_ratio = 18 × 10 = 180
//
//   Fine-tune: if Serial shows 210V but your mains is 225V:
//     new VOLT_RATIO = 180 × (210 / 225) = 168  (lower ratio → higher displayed voltage)
//
// CT_RATIO:
//   Read the label on your CT coil. Examples:
//     "100A / 50mA"  → ratio = 100 / 0.050 = 2000
//     "20A / 10mA"   → ratio = 20  / 0.010 = 2000
//     "SCT-013-030"  → built-in burden, ratio = 30   ← common cheap CT
//   Fine-tune: if Serial shows 0.30A but clamp meter reads 0.45A → multiply ratio by (0.45/0.30) = 1.5
//
// BURDEN_RESISTOR:
//   The physical resistor across your CT secondary. Confirmed: 330 Ohms.
//   (If your CT has a built-in burden, set this to 1.0 and fold the burden into CT_RATIO instead.)
//
// ADC_OFFSET:
//   The ADC value when the AC signal is at 0V (the bias mid-point).
//   Ideal: 2048 (exactly half of 4096). Your circuit biases to 1.65V = 3.3V/2 → offset = 2048.
//   Fine-tune: upload with RAW_DEBUG_MODE 1, read idle voltage channel ADC value with no mains.
//             That value IS your ADC_OFFSET. Update and re-upload.

#define VOLT_RATIO      180.0   // 225V mains ÷ 18:1 transformer ÷ 10:1 divider (9kΩ/1kΩ) = 180
#define CT_RATIO        30.0    // ← UPDATE: read your CT coil label (see formula above)
#define BURDEN_RESISTOR 330.0   // Confirmed: 330 Ohm burden resistor on CT secondary
#define ADC_OFFSET      2048    // Mid-rail bias point (1.65V = 3.3V/2 → 4096/2 = 2048)

// ============= RAW DEBUG MODE =============
// Set to 1 for calibration: prints raw ADC counts + calculated values every loop.
// Set to 0 for normal operation (less serial noise).
// See calibration steps in the header comment above.
#define RAW_DEBUG_MODE  0

// ============= PFC THRESHOLDS =============
#define PF_EXCELLENT_THRESHOLD 0.95f
#define PF_GOOD_THRESHOLD      0.90f
#define PF_ACCEPTABLE_THRESHOLD 0.85f

// ============= BLYNK UPDATE INTERVAL =============
// Blynk is updated every N loop iterations to avoid flooding the server
#define BLYNK_UPDATE_EVERY_N 5

// ============= DATA STRUCTURES =============
struct Sample {
  uint16_t raw_voltage;
  uint16_t raw_current;
  uint32_t timestamp_us;  // Microseconds since capture start
  uint16_t sample_index;  // Position in array (0 to TOTAL_SAMPLES-1)
};

// Simple enum — avoids error-prone String comparisons everywhere
enum PhaseType { IN_PHASE, LAGGING, LEADING };

struct WaveformData {
  Sample samples[TOTAL_SAMPLES];
  uint16_t peak_volt_index;
  uint16_t peak_curr_index;
  uint16_t peak_volt_adc;
  uint16_t peak_curr_adc;
  float voltage_rms;
  float current_rms;
  float real_power;
  float apparent_power;
  float power_factor;
  float phase_angle_deg;
  PhaseType phase_type;  // IN_PHASE, LAGGING, or LEADING
};

struct RelayControl {
  bool relay_5uf = false;
  bool relay_12uf = false;
  bool auto_mode = true;
};

// ============= GLOBAL VARIABLES =============
WaveformData waveform;
RelayControl relay_control;
uint8_t blynk_loop_counter = 0;

// ============= FUNCTION DECLARATIONS =============
void captureFullCycle();
void findPeaks();
void calculatePowerMetrics();
void detectPhase();
void autoCorrectPFC();
void controlRelays();
void updateLCD();
void sendToBlynk();
void printResults();

// ============= SETUP =============
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n========================================");
  Serial.println("   ESP32 Dual ADC PFC Phase Detector");
  Serial.println("========================================\n");

  // ----- I2C LCD -----
  // NOTE: The LCD backlight LED is hard-wired always-on via a solder jumper on the module;
  //       lcd.backlight() is kept here for completeness but has no effect on that hardware.
  Wire.begin();  // SDA = GPIO 21, SCL = GPIO 22 (ESP32 defaults)
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("PFC Controller");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  // ----- ADC -----
  pinMode(VOLT_PIN_ADC1, INPUT);
  pinMode(CURR_PIN_ADC1, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(VOLT_PIN_ADC1, ADC_11db);
  analogSetPinAttenuation(CURR_PIN_ADC1, ADC_11db);

  // ----- Relays & LED -----
  // Relay modules are ACTIVE-LOW: HIGH = relay OFF (de-energized), LOW = relay ON (energized).
  // Initialise both relays HIGH so they are OFF at startup.
  pinMode(RELAY_5UF_PIN, OUTPUT);
  pinMode(RELAY_12UF_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RELAY_5UF_PIN, HIGH);   // OFF at startup (active-low module)
  digitalWrite(RELAY_12UF_PIN, HIGH);  // OFF at startup (active-low module)
  digitalWrite(LED_PIN, LOW);

  // ----- WiFi + Blynk (non-blocking with 10-second timeout) -----
  Serial.print("[SETUP] Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  lcd.setCursor(0, 1);
  lcd.print("WiFi connect... ");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long wifi_start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifi_start < 10000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect(3000);  // 3-second Blynk server connect timeout
    Serial.println("\n[SETUP] WiFi + Blynk connected");
    lcd.setCursor(0, 1);
    lcd.print("Blynk OK!       ");
  } else {
    Serial.println("\n[SETUP] WiFi failed — running standalone");
    lcd.setCursor(0, 1);
    lcd.print("Standalone mode ");
  }
  delay(1000);

  Serial.println("[SETUP] ADC configuration complete");
  Serial.print("[SETUP] Capturing ");
  Serial.print(TOTAL_SAMPLES);
  Serial.print(" samples every ");
  Serial.print(SAMPLE_INTERVAL_US);
  Serial.println("us (1 full cycle)");
  Serial.print("[SETUP] Full-cycle duration: ");
  Serial.print(FULL_CYCLE_TIME_US / 1000UL);
  Serial.println("ms");

  // Print active calibration values so you can verify them at a glance
  Serial.println("\n--- CALIBRATION VALUES ---");
  Serial.print("  VOLT_RATIO:      "); Serial.println(VOLT_RATIO);
  Serial.print("  CT_RATIO:        "); Serial.println(CT_RATIO);
  Serial.print("  BURDEN_RESISTOR: "); Serial.print(BURDEN_RESISTOR); Serial.println(" Ohm");
  Serial.print("  ADC_OFFSET:      "); Serial.println(ADC_OFFSET);
#if RAW_DEBUG_MODE
  Serial.println("  RAW_DEBUG_MODE:  ON  (set to 0 for clean operation)");
#else
  Serial.println("  RAW_DEBUG_MODE:  OFF");
#endif
  Serial.println("--------------------------\n");
}

// ============= MAIN LOOP =============
void loop() {
  Blynk.run();  // Keep Blynk connection alive

  // Capture one complete full cycle with simultaneous dual ADC reads
  captureFullCycle();
  
  // Find peak voltage and current values and their positions
  findPeaks();
  
  // Calculate RMS, real power, apparent power
  calculatePowerMetrics();
  
  // Detect phase relationship using peak positions
  detectPhase();
  
  // Control relays based on power factor and phase type
  if (relay_control.auto_mode) {
    autoCorrectPFC();
  }
  controlRelays();

  // Update I2C LCD with priority display
  updateLCD();

  // Send data to Blynk every N iterations
  blynk_loop_counter++;
  if (blynk_loop_counter >= BLYNK_UPDATE_EVERY_N) {
    blynk_loop_counter = 0;
    sendToBlynk();
  }

  // Display results on serial monitor
  printResults();
  
  delay(500);
}

// ============= CAPTURE ONE FULL CYCLE =============
void captureFullCycle() {
  Serial.println("[CAPTURE] Starting simultaneous dual-ADC capture (full cycle)...");
  
  unsigned long start_time = micros();
  
  for (uint16_t i = 0; i < TOTAL_SAMPLES; i++) {
    // Read both ADC1 channels back-to-back (both on ADC1 — WiFi-safe)
    waveform.samples[i].raw_voltage = analogRead(VOLT_PIN_ADC1);
    waveform.samples[i].raw_current = analogRead(CURR_PIN_ADC1);
    
    // Record timestamp relative to capture start
    waveform.samples[i].timestamp_us = micros() - start_time;
    waveform.samples[i].sample_index = i;
    
    // Wait for next sample interval, compensating for ADC read time
    if (ADJUSTED_DELAY_US > 0) {
      delayMicroseconds(ADJUSTED_DELAY_US);
    }
  }
  
  unsigned long total_time = micros() - start_time;
  Serial.print("[CAPTURE] Complete in ");
  Serial.print(total_time);
  Serial.println("us");

#if RAW_DEBUG_MODE
  // Print the first 5 raw ADC samples so you can check the idle offset
  Serial.println("[RAW] First 5 samples (voltage_adc, current_adc):");
  for (uint8_t d = 0; d < 5; d++) {
    Serial.print("  [");
    Serial.print(d);
    Serial.print("] V=");
    Serial.print(waveform.samples[d].raw_voltage);
    Serial.print("  I=");
    Serial.println(waveform.samples[d].raw_current);
  }
#endif
}

// ============= FIND PEAK VOLTAGE AND CURRENT =============
void findPeaks() {
  uint16_t peak_volt = 0;
  uint16_t peak_curr = 0;
  waveform.peak_volt_index = 0;
  waveform.peak_curr_index = 0;
  
  // Find the highest voltage reading.
  // Cast to int16_t before taking abs() to avoid unsigned underflow
  // when raw ADC value is below ADC_OFFSET.
  for (uint16_t i = 0; i < TOTAL_SAMPLES; i++) {
    uint16_t centered_volt = (uint16_t)abs((int16_t)waveform.samples[i].raw_voltage - (int16_t)ADC_OFFSET);
    if (centered_volt > peak_volt) {
      peak_volt = centered_volt;
      waveform.peak_volt_index = i;
      waveform.peak_volt_adc = waveform.samples[i].raw_voltage;
    }
  }
  
  // Find the highest current reading (same signed-subtraction fix)
  for (uint16_t i = 0; i < TOTAL_SAMPLES; i++) {
    uint16_t centered_curr = (uint16_t)abs((int16_t)waveform.samples[i].raw_current - (int16_t)ADC_OFFSET);
    if (centered_curr > peak_curr) {
      peak_curr = centered_curr;
      waveform.peak_curr_index = i;
      waveform.peak_curr_adc = waveform.samples[i].raw_current;
    }
  }
  
  Serial.print("[PEAKS] Voltage peak at index: ");
  Serial.print(waveform.peak_volt_index);
  Serial.print(" (time: ");
  Serial.print(waveform.samples[waveform.peak_volt_index].timestamp_us);
  Serial.println("us)");
  
  Serial.print("[PEAKS] Current peak at index: ");
  Serial.print(waveform.peak_curr_index);
  Serial.print(" (time: ");
  Serial.print(waveform.samples[waveform.peak_curr_index].timestamp_us);
  Serial.println("us)");
}

// ============= CALCULATE RMS, POWER =============
void calculatePowerMetrics() {
  float sum_volt_sq = 0;
  float sum_curr_sq = 0;
  float sum_real_power = 0;
  
  // Convert ADC readings to actual voltage and current, then calculate RMS
  for (uint16_t i = 0; i < TOTAL_SAMPLES; i++) {
    // Convert raw ADC to centered AC value
    float volt_centered = (waveform.samples[i].raw_voltage - ADC_OFFSET) * (3.3 / 4096.0) / VOLT_RATIO;
    float curr_centered = (waveform.samples[i].raw_current - ADC_OFFSET) * (3.3 / 4096.0) / (BURDEN_RESISTOR * CT_RATIO);
    
    sum_volt_sq += volt_centered * volt_centered;
    sum_curr_sq += curr_centered * curr_centered;
    sum_real_power += volt_centered * curr_centered;
  }
  
  // RMS = sqrt(sum of squares / number of samples)
  waveform.voltage_rms = sqrt(sum_volt_sq / TOTAL_SAMPLES);
  waveform.current_rms = sqrt(sum_curr_sq / TOTAL_SAMPLES);
  waveform.real_power = sum_real_power / TOTAL_SAMPLES;
  waveform.apparent_power = waveform.voltage_rms * waveform.current_rms;
  
  // Power factor = real power / apparent power
  if (waveform.apparent_power > 0.01) {
    waveform.power_factor = waveform.real_power / waveform.apparent_power;
  } else {
    waveform.power_factor = 0;
  }
  
  waveform.power_factor = constrain(waveform.power_factor, -1, 1);
}

// ============= DETECT PHASE (LEADING vs LAGGING) =============
void detectPhase() {
  // Phase angle difference = (index difference / total samples) * 360°
  // One full cycle = 360°
  
  int index_diff = (int)waveform.peak_curr_index - (int)waveform.peak_volt_index;
  
  // Convert index difference to time difference using signed arithmetic
  int32_t time_diff_signed = (int32_t)waveform.samples[waveform.peak_curr_index].timestamp_us -
                              (int32_t)waveform.samples[waveform.peak_volt_index].timestamp_us;
  uint32_t time_diff_us = (uint32_t)abs(time_diff_signed);
  
  // Full cycle time in µs
  uint32_t full_cycle_time_us = FULL_CYCLE_TIME_US;
  
  // Calculate phase angle: (time_diff / full_cycle_time) * 360°
  waveform.phase_angle_deg = (float)time_diff_us / (float)full_cycle_time_us * 360.0f;
  
  // Determine if leading or lagging based on peak index order
  if (index_diff > 0) {
    waveform.phase_type = LAGGING;   // Current peaks AFTER voltage
  } else if (index_diff < 0) {
    waveform.phase_type = LEADING;   // Current peaks BEFORE voltage
  } else {
    waveform.phase_type = IN_PHASE;  // Peaks aligned
  }
  
  // Cross-check: if calculated real power is negative, load is capacitive (leading)
  if (waveform.real_power < 0) {
    waveform.phase_type = LEADING;
  }
  
  // Phase angle from arccos of |PF| for reference
  float pf_angle = acos(fabs(waveform.power_factor)) * 180.0f / (float)M_PI;
  
  Serial.print("[PHASE] Index difference: ");
  Serial.print(index_diff);
  Serial.print(" | Time difference: ");
  Serial.print(time_diff_us);
  Serial.println("us");
  Serial.print("[PHASE] Phase angle (from peaks): ");
  Serial.print(waveform.phase_angle_deg, 2);
  Serial.print("° | Phase angle (from PF): ");
  Serial.print(pf_angle, 2);
  Serial.println("°");
}

// ============= AUTOMATIC PFC CORRECTION =============
void autoCorrectPFC() {
  float pf_abs = fabs(waveform.power_factor);
  
  // Capacitor banks should only be switched in for LAGGING (inductive) loads.
  // For LEADING (capacitive) loads, adding more capacitance worsens the power factor.
  bool is_lagging = (waveform.phase_type == LAGGING);
  
  if (!is_lagging || pf_abs >= PF_EXCELLENT_THRESHOLD) {
    // Power factor is already good, or load is capacitive — turn off all capacitors
    relay_control.relay_5uf = false;
    relay_control.relay_12uf = false;
  }
  else if (pf_abs >= PF_GOOD_THRESHOLD) {
    // Mild lagging: 5µF correction only
    relay_control.relay_5uf = true;
    relay_control.relay_12uf = false;
  }
  else {
    // PF is below GOOD threshold (< 0.90) — use both capacitor banks for maximum correction
    relay_control.relay_5uf = true;
    relay_control.relay_12uf = true;
  }
}

// ============= CONTROL RELAYS =============
// Relay modules are ACTIVE-LOW: LOW energizes the relay (ON), HIGH de-energizes it (OFF).
void controlRelays() {
  digitalWrite(RELAY_5UF_PIN,  relay_control.relay_5uf  ? LOW : HIGH);
  digitalWrite(RELAY_12UF_PIN, relay_control.relay_12uf ? LOW : HIGH);
}

// ============= UPDATE I2C LCD =============
// Overwrites each line in-place (no lcd.clear → no flicker).
// Line 1: "PF:0.97 LAGGING " — power factor + phase direction
// Line 2: "5uF R1  12uF R2 " or "Caps: OFF       "
void updateLCD() {
  char line[17];  // 16 visible chars + null terminator

  // --- Line 1: PF and phase direction (always fits in 16 chars) ---
  const char* phase_label;
  if      (waveform.phase_type == LAGGING)  phase_label = "LAGGING ";
  else if (waveform.phase_type == LEADING)  phase_label = "LEADING ";
  else                                       phase_label = "IN PHASE";
  snprintf(line, sizeof(line), "PF:%.2f %s", fabs(waveform.power_factor), phase_label);
  lcd.setCursor(0, 0);
  lcd.print(line);

  // --- Line 2: Active capacitor relay(s), max 16 chars ---
  if (!relay_control.relay_5uf && !relay_control.relay_12uf) {
    snprintf(line, sizeof(line), "Caps: OFF       ");
  } else if (relay_control.relay_5uf && relay_control.relay_12uf) {
    snprintf(line, sizeof(line), "5uF R1 + 12uF R2");  // exactly 16
  } else if (relay_control.relay_5uf) {
    snprintf(line, sizeof(line), "5uF R1 ON       ");
  } else {
    snprintf(line, sizeof(line), "12uF R2 ON      ");
  }
  lcd.setCursor(0, 1);
  lcd.print(line);
}

// ============= SEND DATA TO BLYNK =============
void sendToBlynk() {
  if (!Blynk.connected()) return;  // Skip if offline — runs fine standalone
  Blynk.virtualWrite(VPIN_VOLTAGE,       waveform.voltage_rms);
  Blynk.virtualWrite(VPIN_POWER_FACTOR,  fabs(waveform.power_factor));
  Blynk.virtualWrite(VPIN_PHASE_ANGLE,   waveform.phase_angle_deg);
  Blynk.virtualWrite(VPIN_PHASE_TYPE,    (int)waveform.phase_type);  // 0=IN_PHASE 1=LAGGING 2=LEADING
  Blynk.virtualWrite(VPIN_RELAY_5UF,     relay_control.relay_5uf  ? 1 : 0);
  Blynk.virtualWrite(VPIN_RELAY_12UF,    relay_control.relay_12uf ? 1 : 0);
}


void printResults() {
  Serial.println("\n========== POWER FACTOR ANALYSIS ==========");
  Serial.print("Voltage RMS: ");
  Serial.print(waveform.voltage_rms, 2);
  Serial.println(" V");
  
  Serial.print("Current RMS: ");
  Serial.print(waveform.current_rms, 3);
  Serial.println(" A");
  
  Serial.print("Real Power: ");
  Serial.print(waveform.real_power, 2);
  Serial.println(" W");
  
  Serial.print("Apparent Power: ");
  Serial.print(waveform.apparent_power, 2);
  Serial.println(" VA");
  
  Serial.print("Power Factor: ");
  Serial.print(fabs(waveform.power_factor), 3);
  Serial.print(" (");
  
  if (fabs(waveform.power_factor) >= PF_EXCELLENT_THRESHOLD) {
    Serial.print("EXCELLENT");
  } else if (fabs(waveform.power_factor) >= PF_GOOD_THRESHOLD) {
    Serial.print("GOOD");
  } else if (fabs(waveform.power_factor) >= PF_ACCEPTABLE_THRESHOLD) {
    Serial.print("ACCEPTABLE");
  } else {
    Serial.print("POOR");
  }
  Serial.println(")");
  
  Serial.print("Phase Angle: ");
  Serial.print(waveform.phase_angle_deg, 1);
  Serial.print(" deg (");
  if      (waveform.phase_type == LAGGING)  Serial.print("LAGGING");
  else if (waveform.phase_type == LEADING)  Serial.print("LEADING");
  else                                       Serial.print("IN_PHASE");
  Serial.println(")");
  
  Serial.print("Relays: ");
  if (!relay_control.relay_5uf && !relay_control.relay_12uf) {
    Serial.print("OFF");
  } else {
    if (relay_control.relay_5uf)  Serial.print("5µF ");
    if (relay_control.relay_12uf) Serial.print("12µF");
  }
  Serial.println();
  
  Serial.println("=========================================\n");
}
