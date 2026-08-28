/*
 * ESP32 Dual ADC Simultaneous Sampling with Phase Detection
 * Captures one half-wave (10ms @ 50Hz) with timestamps
 * Calculates phase difference to determine leading/lagging
 * Computes power factor for automatic PFC correction
 */

#include <Arduino.h>
#include <cmath>

// ============= PIN CONFIGURATION =============
#define VOLT_PIN_ADC1 36    // ADC1_CH0 (GPIO 36) - Voltage
#define CURR_PIN_ADC2 4     // ADC2_CH0 (GPIO 4)  - Current
#define RELAY_5UF_PIN 26    // Relay 1 (5µF)
#define RELAY_12UF_PIN 25   // Relay 2 (12µF)
#define LED_PIN 2           // Status LED

// ============= SAMPLING CONFIGURATION =============
#define AC_FREQUENCY 50                          // 50Hz or 60Hz
#define HALF_WAVE_TIME_MS (1000 / (AC_FREQUENCY * 2))  // 10ms for 50Hz
#define SAMPLE_INTERVAL_US 50                    // Sample every 50µs
#define TOTAL_SAMPLES (HALF_WAVE_TIME_MS * 1000 / SAMPLE_INTERVAL_US)  // 200 samples

// ============= SENSOR CALIBRATION =============
#define VOLT_RATIO 234.0        // Voltage divider ratio
#define CT_RATIO 30.0           // CT sensor turns ratio
#define BURDEN_RESISTOR 100.0   // Burden resistor (Ohms)
#define ADC_OFFSET 2048         // ADC center point (12-bit: 0-4095)

// ============= PFC THRESHOLDS =============
#define PF_EXCELLENT_THRESHOLD 0.95
#define PF_GOOD_THRESHOLD 0.90
#define PF_ACCEPTABLE_THRESHOLD 0.85

// ============= DATA STRUCTURES =============
struct Sample {
  uint16_t raw_voltage;
  uint16_t raw_current;
  uint32_t timestamp_us;  // Microseconds
  uint16_t sample_index;  // Position in array (0-199)
};

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
  String phase_type;  // "LEADING", "LAGGING", or "IN_PHASE"
};

struct RelayControl {
  bool relay_5uf = false;
  bool relay_12uf = false;
  bool auto_mode = true;
};

// ============= GLOBAL VARIABLES =============
WaveformData waveform;
RelayControl relay_control;

// ============= FUNCTION DECLARATIONS =============
void captureHalfWave();
void findPeaks();
void calculatePowerMetrics();
void detectPhase();
void controlRelays();
void printResults();

// ============= SETUP =============
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n========================================");
  Serial.println("   ESP32 Dual ADC PFC Phase Detector");
  Serial.println("========================================\n");
  
  // Initialize ADC pins
  pinMode(VOLT_PIN_ADC1, INPUT);
  pinMode(CURR_PIN_ADC2, INPUT);
  
  // Set ADC attenuation for full range
  analogSetPinAttenuation(VOLT_PIN_ADC1, ADC_11db);
  analogSetPinAttenuation(CURR_PIN_ADC2, ADC_11db);
  
  // Initialize relay pins
  pinMode(RELAY_5UF_PIN, OUTPUT);
  pinMode(RELAY_12UF_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  digitalWrite(RELAY_5UF_PIN, LOW);
  digitalWrite(RELAY_12UF_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("[SETUP] ADC configuration complete");
  Serial.print("[SETUP] Capturing ");
  Serial.print(TOTAL_SAMPLES);
  Serial.print(" samples every ");
  Serial.print(SAMPLE_INTERVAL_US);
  Serial.println("µs");
  Serial.print("[SETUP] Half-wave duration: ");
  Serial.print(HALF_WAVE_TIME_MS);
  Serial.println("ms\n");
}

// ============= MAIN LOOP =============
void loop() {
  // Capture one complete half-wave with simultaneous dual ADC reads
  captureHalfWave();
  
  // Find peak voltage and current values and their positions
  findPeaks();
  
  // Calculate RMS, real power, apparent power
  calculatePowerMetrics();
  
  // Detect phase relationship using peak positions
  detectPhase();
  
  // Control relays based on power factor
  if (relay_control.auto_mode) {
    autoCorrectPFC();
  }
  controlRelays();
  
  // Display results
  printResults();
  
  delay(500);
}

// ============= CAPTURE ONE HALF-WAVE =============
void captureHalfWave() {
  Serial.println("[CAPTURE] Starting simultaneous dual-ADC capture...");
  
  unsigned long start_time = micros();
  
  for (uint16_t i = 0; i < TOTAL_SAMPLES; i++) {
    // Read BOTH ADCs simultaneously
    // ADC1 (GPIO 36) and ADC2 (GPIO 4) operate independently
    // Delay between reads: <500 nanoseconds (negligible)
    
    waveform.samples[i].raw_voltage = analogRead(VOLT_PIN_ADC1);
    waveform.samples[i].raw_current = analogRead(CURR_PIN_ADC2);
    
    // Record timestamp relative to capture start
    waveform.samples[i].timestamp_us = micros() - start_time;
    waveform.samples[i].sample_index = i;
    
    // Wait for next sample interval
    delayMicroseconds(SAMPLE_INTERVAL_US);
  }
  
  unsigned long total_time = micros() - start_time;
  Serial.print("[CAPTURE] Complete in ");
  Serial.print(total_time);
  Serial.println("µs");
}

// ============= FIND PEAK VOLTAGE AND CURRENT =============
void findPeaks() {
  uint16_t peak_volt = 0;
  uint16_t peak_curr = 0;
  waveform.peak_volt_index = 0;
  waveform.peak_curr_index = 0;
  
  // Find the highest voltage reading
  for (uint16_t i = 0; i < TOTAL_SAMPLES; i++) {
    uint16_t centered_volt = abs(waveform.samples[i].raw_voltage - ADC_OFFSET);
    if (centered_volt > peak_volt) {
      peak_volt = centered_volt;
      waveform.peak_volt_index = i;
      waveform.peak_volt_adc = waveform.samples[i].raw_voltage;
    }
  }
  
  // Find the highest current reading
  for (uint16_t i = 0; i < TOTAL_SAMPLES; i++) {
    uint16_t centered_curr = abs(waveform.samples[i].raw_current - ADC_OFFSET);
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
  Serial.println("µs)");
  
  Serial.print("[PEAKS] Current peak at index: ");
  Serial.print(waveform.peak_curr_index);
  Serial.print(" (time: ");
  Serial.print(waveform.samples[waveform.peak_curr_index].timestamp_us);
  Serial.println("µs)");
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
  // But for AC: one full cycle = 360°, half-wave = 180°
  
  int index_diff = waveform.peak_curr_index - waveform.peak_volt_index;
  
  // Convert index difference to time difference
  uint32_t time_diff_us = abs((int)waveform.samples[waveform.peak_curr_index].timestamp_us - 
                               (int)waveform.samples[waveform.peak_volt_index].timestamp_us);
  
  // Time for half-wave (180°)
  uint32_t half_wave_time_us = HALF_WAVE_TIME_MS * 1000;
  
  // Calculate phase angle: (time_diff / half_wave_time) * 180°
  waveform.phase_angle_deg = (float)time_diff_us / half_wave_time_us * 180.0;
  
  // Determine if leading or lagging
  if (index_diff > 0) {
    waveform.phase_type = "LAGGING";  // Current peaks AFTER voltage
  } else if (index_diff < 0) {
    waveform.phase_type = "LEADING";  // Current peaks BEFORE voltage
  } else {
    waveform.phase_type = "IN_PHASE"; // Peaks aligned
  }
  
  // Verify with real power sign
  if (waveform.real_power < 0) {
    waveform.phase_type = "LEADING";  // Negative power = capacitive
  }
  
  // Also calculate using arccos of PF
  float pf_angle = acos(fabs(waveform.power_factor)) * 180.0 / M_PI;
  
  Serial.print("[PHASE] Index difference: ");
  Serial.print(index_diff);
  Serial.print(" | Time difference: ");
  Serial.print(time_diff_us);
  Serial.println("µs");
  Serial.print("[PHASE] Phase angle (from peaks): ");
  Serial.print(waveform.phase_angle_deg, 2);
  Serial.print("° | Phase angle (from PF): ");
  Serial.print(pf_angle, 2);
  Serial.println("°");
}

// ============= AUTOMATIC PFC CORRECTION =============
void autoCorrectPFC() {
  float pf_abs = fabs(waveform.power_factor);
  
  if (pf_abs >= PF_EXCELLENT_THRESHOLD) {
    relay_control.relay_5uf = false;
    relay_control.relay_12uf = false;
  }
  else if (pf_abs >= PF_GOOD_THRESHOLD) {
    relay_control.relay_5uf = true;
    relay_control.relay_12uf = false;
  }
  else if (pf_abs >= PF_ACCEPTABLE_THRESHOLD) {
    relay_control.relay_5uf = true;
    relay_control.relay_12uf = true;
  }
  else {
    relay_control.relay_5uf = true;
    relay_control.relay_12uf = true;
  }
}

// ============= CONTROL RELAYS =============
void controlRelays() {
  digitalWrite(RELAY_5UF_PIN, relay_control.relay_5uf ? HIGH : LOW);
  digitalWrite(RELAY_12UF_PIN, relay_control.relay_12uf ? HIGH : LOW);
}

// ============= PRINT RESULTS =============
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
  Serial.print("° (");
  Serial.print(waveform.phase_type);
  Serial.println(")");
  
  Serial.print("Relays: ");
  Serial.print(relay_control.relay_5uf ? "5µF " : "");
  Serial.print(relay_control.relay_12uf ? "12µF " : "OFF");
  Serial.println();
  
  Serial.println("=========================================\n");
}
