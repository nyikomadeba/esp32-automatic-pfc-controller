# ESP32 Dual ADC Phase Detection for Power Factor Correction

## How It Works

This firmware captures a complete half-wave (10ms @ 50Hz) using **simultaneous dual ADC sampling** and performs phase detection in 3 simple steps:

### Step 1: Simultaneous Dual ADC Capture (10ms)
```
Voltage peak timing: sample[48]  @ 2400µs
Current peak timing: sample[65]  @ 3250µs

Time difference: 850µs = 15.3° phase shift
```

### Step 2: Find Peak Voltage & Current
- Scans all 200 samples
- Records **index** of highest voltage
- Records **index** of highest current
- Gets **timestamp** of each peak from array

### Step 3: Calculate Phase Difference
```
Phase angle = (time_diff / half_wave_duration) × 180°
            = (850µs / 10000µs) × 180°
            = 15.3°

cos(15.3°) = 0.964 = Power Factor
```

## Pin Configuration

| Pin | GPIO | ADC | Use |
|-----|------|-----|-----|
| Voltage | 36 | ADC1_CH0 | Voltage measurement |
| Current | 4 | ADC2_CH0 | Current measurement |
| Relay 1 | 26 | - | 5µF capacitor |
| Relay 2 | 25 | - | 12µF capacitor |
| LED | 2 | - | Status indicator |

## Sampling Details

```
Sampling Interval: 50µs
Total Samples: 200
Capture Duration: 10ms (half-wave @ 50Hz)

Timestamps:
Sample 0:   0µs     (voltage = V_peak)
Sample 48:  2400µs  (voltage = V_peak)  ← Peak voltage here
Sample 65:  3250µs  (current = I_peak)  ← Peak current here  (LAGGING)
Sample 199: 9950µs  (end of half-wave)

Phase difference = 850µs = 15.3° lagging
```

## Algorithm Flow

```
1. captureHalfWave()
   ├─ Loop 200 times
   ├─ Read ADC1 (voltage) & ADC2 (current) SIMULTANEOUSLY
   ├─ Store in array with timestamp & index
   └─ Wait 50µs until next sample

2. findPeaks()
   ├─ Scan voltage array → find highest → store index (peak_volt_index)
   ├─ Scan current array → find highest → store index (peak_curr_index)
   └─ Get timestamps from arrays

3. calculatePowerMetrics()
   ├─ Convert each ADC to voltage/current
   ├─ Calculate sum of squares
   ├─ RMS = √(sum / samples)
   ├─ Real power = sum(V×I) / samples
   ├─ Apparent power = V_RMS × I_RMS
   └─ Power factor = Real / Apparent

4. detectPhase()
   ├─ time_diff = timestamp[current_peak] - timestamp[voltage_peak]
   ├─ phase_angle = (time_diff / 10000µs) × 180°
   ├─ If current peak AFTER voltage peak → LAGGING
   ├─ If current peak BEFORE voltage peak → LEADING
   └─ Phase type determines capacitor need

5. autoCorrectPFC()
   ├─ If PF ≥ 0.95 → Turn OFF all capacitors
   ├─ If 0.90 ≤ PF < 0.95 → Turn ON 5µF only
   ├─ If 0.85 ≤ PF < 0.90 → Turn ON 5µF + 12µF
   └─ If PF < 0.85 → Keep 5µF + 12µF on (max correction)
```

## Serial Output Example

```
[CAPTURE] Starting simultaneous dual-ADC capture...
[CAPTURE] Complete in 10048µs
[PEAKS] Voltage peak at index: 48 (time: 2400µs)
[PEAKS] Current peak at index: 65 (time: 3250µs)
[PHASE] Index difference: 17 | Time difference: 850µs
[PHASE] Phase angle (from peaks): 15.30° | Phase angle (from PF): 15.27°

========== POWER FACTOR ANALYSIS ==========
Voltage RMS: 230.45 V
Current RMS: 2.345 A
Real Power: 528.40 W
Apparent Power: 541.18 VA
Power Factor: 0.976 (EXCELLENT)
Phase Angle: 15.3° (LAGGING)
Relays: 5µF
=========================================
```

## Key Concepts

### Why Dual ADC?
- **ADC1** (GPIO 36) reads voltage independently
- **ADC2** (GPIO 4) reads current independently
- Both ADCs operate on **different clock domains**
- Reads are separated by <500 nanoseconds (negligible)

### Why Peak Detection?
- Peak voltage occurs at exactly one point per half-wave
- Peak current occurs at exactly one point per half-wave
- The **time difference** between peaks = **phase shift**
- This is more accurate than zero-crossing detection

### Phase Relationship
```
LAGGING (Inductive load - typical):
Voltage ─╱╲─────╱╲─────
          
Current ──╱╲───╱╲───  (delayed)
         ↑ Current lags voltage

LEADING (Capacitive load - rare):
Voltage ──╱╲───╱╲───
          
Current ─╱╲─────╱╲─  (early)
         ↑ Current leads voltage

IN PHASE (Purely resistive):
Voltage ─╱╲─────╱╲─
Current ─╱╲─────╱╲─  (perfectly aligned)
```

## Power Factor Correction Strategy

```
Lagging current (typical motors, inductors):
- Add capacitors (C) in parallel
- Capacitors draw leading current
- Leading current cancels lagging current
- Net phase angle → 0° → PF → 1.0

Capacitor Banks:
5µF    @ 230V ≈ 0.36 kVAR (light correction)
12µF   @ 230V ≈ 0.87 kVAR (medium correction)
17µF   @ 230V ≈ 1.23 kVAR (strong correction)
```

## Calibration

If readings are inaccurate:

```cpp
// Adjust voltage divider ratio
#define VOLT_RATIO 234.0  // Measure actual voltage, adjust until correct

// Adjust CT sensor ratio and burden resistor
#define CT_RATIO 30.0              // Your CT turns ratio
#define BURDEN_RESISTOR 100.0      // Resistor across CT secondary
```

## Advantages Over Traditional Methods

| Method | Accuracy | Speed | Phase Detection |
|--------|----------|-------|------------------|
| Zero-crossing | Good | Fast | Moderate |
| **Peak Detection (This)** | **Excellent** | **Fast** | **Excellent** |
| FFT Analysis | Excellent | Slow | Excellent |
| Phaser Measurement | Best | Slowest | Perfect |

## Future Improvements

- [ ] Add harmonics detection
- [ ] WiFi dashboard for monitoring
- [ ] Data logging to SD card
- [ ] MQTT integration
- [ ] Predictive PFC adjustment

## Author

Built for automatic power factor correction on ESP32
