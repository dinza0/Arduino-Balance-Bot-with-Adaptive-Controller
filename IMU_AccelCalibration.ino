/*
 * IMU_AccelCalibration.ino
 * 
 * Standalone accelerometer calibration and CSV data logger
 * for MPU-6050 on Arduino Mega R3
 * 
 * Hardware:
 *   MPU-6050 SDA -> Pin 20
 *   MPU-6050 SCL -> Pin 21
 *   MPU-6050 VCC -> 3.3V or 5V
 *   MPU-6050 GND -> GND
 *   MPU-6050 AD0 -> GND (I2C address 0x68)
 * 
 * Serial Commands:
 *   c = calibrate (keep robot still)
 *   l = start 60s log at 10Hz (auto-dumps CSV when done)
 *   q = stop log early and dump now
 *   p = print current live readings
 *   r = reset calibration offsets
 *   h = help menu
 */

#include <Wire.h>
#include <math.h>

// ================================================================
// CONFIGURATION
// ================================================================
#define DEBUG_SERIAL    Serial
const uint32_t DEBUG_BAUD = 115200;

// MPU-6050 I2C address
#define MPU_ADDR        0x68

// Calibration samples
#define CAL_SAMPLES     2000

// ================================================================
// RAW IMU VARIABLES
// ================================================================
int16_t RawAccXLSB = 0;
int16_t RawAccYLSB = 0;
int16_t RawAccZLSB = 0;
int16_t RawGyroXLSB = 0;
int16_t RawGyroYLSB = 0;
int16_t RawGyroZLSB = 0;

// Converted values
float AccX = 0.0f;
float AccY = 0.0f;
float AccZ = 0.0f;
float RateRoll  = 0.0f;
float RatePitch = 0.0f;
float RateYaw   = 0.0f;
float AngleRoll  = 0.0f;
float AnglePitch = 0.0f;

bool imu_accel_read_ok = false;
bool imu_gyro_read_ok  = false;

// ================================================================
// CALIBRATION OFFSETS
// ================================================================
// Accelerometer offsets (in G) — subtracted after conversion
float AccX_offset = 0.0f;
float AccY_offset = 0.0f;
float AccZ_offset = 0.0f;  // Z should read 1.0g when flat — offset = measured - 1.0

// Gyro offsets (deg/s) — subtracted after conversion
float RateRoll_offset  = 0.0f;
float RatePitch_offset = 0.0f;
float RateYaw_offset   = 0.0f;

bool calibrated = false;

// ================================================================
// CSV LOGGER
// ================================================================
#define LOG_HZ          10
#define LOG_DURATION_S  30
#define LOG_MAX_SAMPLES 300

struct AccelSample {
  uint16_t time_ms_div10;   // timestamp / 10
  int16_t  accX_raw;        // AccX * 1000 (after calibration)
  int16_t  accY_raw;
  int16_t  accZ_raw;
  int16_t  rateRoll_raw;    // RateRoll * 10 (after calibration)
  int16_t  ratePitch_raw;
  int16_t  rateYaw_raw;
  int16_t  angleRoll_raw;   // AngleRoll * 10
  int16_t  anglePitch_raw;
};
// 18 bytes × 600 = 10800 bytes
// Reduce LOG_MAX_SAMPLES to 400 if low memory warning appears

AccelSample accel_log[LOG_MAX_SAMPLES];
uint16_t    accel_log_count    = 0;
bool        accel_log_active   = false;
uint32_t    accel_log_last_ms  = 0;
uint32_t    accel_log_start_ms = 0;

const uint32_t LOG_INTERVAL_MS = 1000 / LOG_HZ;

// ================================================================
// MPU-6050 INIT
// ================================================================
void initMPU6050() {
  Wire.begin();
  Wire.setClock(400000);  // 400kHz fast mode
  Wire.setWireTimeout(3000, true);
  delay(250);

  // Wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Low pass filter — register 0x1A, value 0x05 (~10Hz cutoff)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Accelerometer range +/- 8g → sensitivity 4096 LSB/g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Gyroscope range +/- 500 dps → sensitivity 65.5 LSB/(deg/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  delay(100);
  DEBUG_SERIAL.println(F("MPU-6050 initialized."));
  DEBUG_SERIAL.println(F("  Accel range: +/- 8g  (4096 LSB/g)"));
  DEBUG_SERIAL.println(F("  Gyro range:  +/- 500 dps (65.5 LSB/dps)"));
  DEBUG_SERIAL.println(F("  LPF: 0x05 (~10Hz cutoff)"));
}

// ================================================================
// READ RAW IMU
// ================================================================
void readIMU() {
  imu_accel_read_ok = false;
  imu_gyro_read_ok  = false;

  // ── Accelerometer (register 0x3B) ──────────────────────────
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  if (Wire.available() >= 6) {
    RawAccXLSB = (int16_t)((Wire.read() << 8) | Wire.read());
    RawAccYLSB = (int16_t)((Wire.read() << 8) | Wire.read());
    RawAccZLSB = (int16_t)((Wire.read() << 8) | Wire.read());
    imu_accel_read_ok = true;

    // Convert to G
    //AccX = (float)RawAccXLSB / 4096.0f;
    //AccY = (float)RawAccYLSB / 4096.0f;
    //AccZ = (float)RawAccZLSB / 4096.0f;
    
    // Calibrated??
    AccX = (float)RawAccXLSB / 4096.0f - 0.047306;
    AccY = (float)RawAccYLSB / 4096.0f - 0.023904;
    AccZ = (float)RawAccZLSB / 4096.0f + 0.008631;

    // Apply calibration offsets
    AccX -= AccX_offset;
    AccY -= AccY_offset;
    AccZ -= AccZ_offset;
  } else {
    while (Wire.available()) Wire.read();
  }

  // ── Gyroscope (register 0x43) ───────────────────────────────
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  if (Wire.available() >= 6) {
    RawGyroXLSB = (int16_t)((Wire.read() << 8) | Wire.read());
    RawGyroYLSB = (int16_t)((Wire.read() << 8) | Wire.read());
    RawGyroZLSB = (int16_t)((Wire.read() << 8) | Wire.read());
    imu_gyro_read_ok = true;

    // Convert to deg/s
    RateRoll  = (float)RawGyroXLSB / 65.5f;
    RatePitch = (float)RawGyroYLSB / 65.5f;
    RateYaw   = (float)RawGyroZLSB / 65.5f;

    // Apply calibration offsets
    RateRoll  -= RateRoll_offset;
    RatePitch -= RatePitch_offset;
    RateYaw   -= RateYaw_offset;
  } else {
    while (Wire.available()) Wire.read();
  }

  // ── Tilt angles ─────────────────────────────────────────────
  if (imu_accel_read_ok) {
    AngleRoll  =  atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180.0f / PI);
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180.0f / PI);
  }
}

// ================================================================
// CALIBRATION
// Keep robot perfectly still on flat surface
// Robot must be in its normal upright operating orientation
// ================================================================
void runCalibration() {
  DEBUG_SERIAL.println(F("================================================"));
  DEBUG_SERIAL.println(F("  ACCELEROMETER CALIBRATION                    "));
  DEBUG_SERIAL.println(F("================================================"));
  DEBUG_SERIAL.println(F("  Place robot on flat level surface"));
  DEBUG_SERIAL.println(F("  Keep PERFECTLY STILL during calibration"));
  DEBUG_SERIAL.println(F("  Starting in 3 seconds..."));
  DEBUG_SERIAL.println(F("================================================"));

  // Blink LED during countdown
  for (int i = 3; i > 0; i--) {
    DEBUG_SERIAL.print(F("  ")); DEBUG_SERIAL.print(i); DEBUG_SERIAL.println(F("..."));
    digitalWrite(13, HIGH); delay(500);
    digitalWrite(13, LOW);  delay(500);
  }

  DEBUG_SERIAL.println(F("  Calibrating — DO NOT MOVE..."));
  digitalWrite(13, HIGH);  // LED on during calibration

  // Accumulate readings
  double sumAccX = 0, sumAccY = 0, sumAccZ = 0;
  double sumRateRoll = 0, sumRatePitch = 0, sumRateYaw = 0;

  // Temporary zero offsets for raw reading
  AccX_offset = 0.0f;
  AccY_offset = 0.0f;
  AccZ_offset = 0.0f;
  RateRoll_offset  = 0.0f;
  RatePitch_offset = 0.0f;
  RateYaw_offset   = 0.0f;

  for (int i = 0; i < CAL_SAMPLES; i++) {
    readIMU();
    sumAccX     += AccX;
    sumAccY     += AccY;
    sumAccZ     += AccZ;
    sumRateRoll  += RateRoll;
    sumRatePitch += RatePitch;
    sumRateYaw   += RateYaw;
    delay(1);

    // Progress every 500 samples
    if (i % 500 == 499) {
      DEBUG_SERIAL.print(F("  ")); 
      DEBUG_SERIAL.print((i+1) * 100 / CAL_SAMPLES);
      DEBUG_SERIAL.println(F("%"));
    }
  }

  // Compute averages
  float meanAccX = (float)(sumAccX  / CAL_SAMPLES);
  float meanAccY = (float)(sumAccY  / CAL_SAMPLES);
  float meanAccZ = (float)(sumAccZ  / CAL_SAMPLES);
  float meanRateRoll  = (float)(sumRateRoll  / CAL_SAMPLES);
  float meanRatePitch = (float)(sumRatePitch / CAL_SAMPLES);
  float meanRateYaw   = (float)(sumRateYaw   / CAL_SAMPLES);

  // Accelerometer offsets:
  // X and Y should read 0g when flat → offset = mean
  // Z should read 1.0g when flat → offset = mean - 1.0
  AccX_offset = meanAccX;
  AccY_offset = meanAccY;
  AccZ_offset = meanAccZ - 1.0f;

  // Gyro offsets — should read 0 when still
  RateRoll_offset  = meanRateRoll;
  RatePitch_offset = meanRatePitch;
  RateYaw_offset   = meanRateYaw;

  calibrated = true;
  digitalWrite(13, LOW);  // LED off

  // Print results
  DEBUG_SERIAL.println(F("================================================"));
  DEBUG_SERIAL.println(F("  CALIBRATION COMPLETE                         "));
  DEBUG_SERIAL.println(F("================================================"));
  DEBUG_SERIAL.println(F("  Accelerometer offsets (G):"));
  DEBUG_SERIAL.print(F("    AccX offset: ")); DEBUG_SERIAL.println(AccX_offset, 6);
  DEBUG_SERIAL.print(F("    AccY offset: ")); DEBUG_SERIAL.println(AccY_offset, 6);
  DEBUG_SERIAL.print(F("    AccZ offset: ")); DEBUG_SERIAL.println(AccZ_offset, 6);
  DEBUG_SERIAL.println(F("  Gyro offsets (deg/s):"));
  DEBUG_SERIAL.print(F("    RateRoll  offset: ")); DEBUG_SERIAL.println(RateRoll_offset, 6);
  DEBUG_SERIAL.print(F("    RatePitch offset: ")); DEBUG_SERIAL.println(RatePitch_offset, 6);
  DEBUG_SERIAL.print(F("    RateYaw   offset: ")); DEBUG_SERIAL.println(RateYaw_offset, 6);
  DEBUG_SERIAL.println(F("------------------------------------------------"));
  DEBUG_SERIAL.println(F("  Copy these into your control code:"));
  DEBUG_SERIAL.println(F("------------------------------------------------"));
  DEBUG_SERIAL.print(F("  AccX = (float)RawAccXLSB / 4096.0f - "));
  DEBUG_SERIAL.print(AccX_offset, 6); DEBUG_SERIAL.println(F(";"));
  DEBUG_SERIAL.print(F("  AccY = (float)RawAccYLSB / 4096.0f - "));
  DEBUG_SERIAL.print(AccY_offset, 6); DEBUG_SERIAL.println(F(";"));
  DEBUG_SERIAL.print(F("  AccZ = (float)RawAccZLSB / 4096.0f - "));
  DEBUG_SERIAL.print(AccZ_offset, 6); DEBUG_SERIAL.println(F(";"));
  DEBUG_SERIAL.println(F("================================================"));

  // Verify — take 10 readings and show corrected values
  DEBUG_SERIAL.println(F("  Verification (should be ~0, ~0, ~1.0):"));
  DEBUG_SERIAL.println(F("  AccX_cal   AccY_cal   AccZ_cal"));
  for (int i = 0; i < 10; i++) {
    readIMU();
    DEBUG_SERIAL.print(F("  "));
    DEBUG_SERIAL.print(AccX, 4); DEBUG_SERIAL.print(F("g    "));
    DEBUG_SERIAL.print(AccY, 4); DEBUG_SERIAL.print(F("g    "));
    DEBUG_SERIAL.print(AccZ, 4); DEBUG_SERIAL.println(F("g"));
    delay(100);
  }
  DEBUG_SERIAL.println(F("================================================"));
}

// ================================================================
// CSV LOGGER FUNCTIONS
// ================================================================
void accelLogStart() {
  accel_log_count    = 0;
  accel_log_active   = true;
  accel_log_start_ms = millis();
  accel_log_last_ms  = millis();

  DEBUG_SERIAL.println(F("================================================"));
  DEBUG_SERIAL.println(F("  IMU LOG STARTED                              "));
  DEBUG_SERIAL.print(F("  Duration: ")); DEBUG_SERIAL.print(LOG_DURATION_S);
  DEBUG_SERIAL.println(F(" seconds"));
  DEBUG_SERIAL.print(F("  Sample rate: ")); DEBUG_SERIAL.print(LOG_HZ);
  DEBUG_SERIAL.println(F(" Hz"));
  DEBUG_SERIAL.print(F("  Max samples: ")); DEBUG_SERIAL.println(LOG_MAX_SAMPLES);
  if (!calibrated) {
    DEBUG_SERIAL.println(F("  WARNING: Not calibrated! Run 'c' first."));
  }
  DEBUG_SERIAL.println(F("  CSV auto-dumps when complete."));
  DEBUG_SERIAL.println(F("  Type 'q' to stop early and dump now."));
  DEBUG_SERIAL.println(F("================================================"));
}

void accelLogUpdate() {
  if (!accel_log_active) return;

  uint32_t now_ms = millis();

  // Auto-dump when full
  if (accel_log_count >= LOG_MAX_SAMPLES) {
    accel_log_active = false;
    DEBUG_SERIAL.println(F("Log complete — auto-dumping CSV..."));
    delay(200);
    accelLogDumpCSV();
    return;
  }

  // Rate limit
  if ((uint32_t)(now_ms - accel_log_last_ms) < LOG_INTERVAL_MS) return;
  accel_log_last_ms = now_ms;

  // Save sample
  AccelSample &s     = accel_log[accel_log_count];
  s.time_ms_div10    = (uint16_t)((now_ms - accel_log_start_ms) / 10);
  s.accX_raw         = (int16_t)(AccX      * 1000.0f);
  s.accY_raw         = (int16_t)(AccY      * 1000.0f);
  s.accZ_raw         = (int16_t)(AccZ      * 1000.0f);
  s.rateRoll_raw     = (int16_t)(RateRoll  * 10.0f);
  s.ratePitch_raw    = (int16_t)(RatePitch * 10.0f);
  s.rateYaw_raw      = (int16_t)(RateYaw   * 10.0f);
  s.angleRoll_raw    = (int16_t)(AngleRoll  * 10.0f);
  s.anglePitch_raw   = (int16_t)(AnglePitch * 10.0f);
  accel_log_count++;

  // Progress every 10 seconds
  if (accel_log_count % (LOG_HZ * 10) == 0) {
    uint32_t elapsed   = (now_ms - accel_log_start_ms) / 1000;
    uint32_t remaining = LOG_DURATION_S - elapsed;
    DEBUG_SERIAL.print(F("Logging... "));
    DEBUG_SERIAL.print(elapsed);
    DEBUG_SERIAL.print(F("s / "));
    DEBUG_SERIAL.print(LOG_DURATION_S);
    DEBUG_SERIAL.print(F("s  ("));
    DEBUG_SERIAL.print(remaining);
    DEBUG_SERIAL.println(F("s remaining)"));
  }
}

void accelLogStopAndDump() {
  accel_log_active = false;
  DEBUG_SERIAL.print(F("Log stopped. Samples: "));
  DEBUG_SERIAL.println(accel_log_count);
  delay(200);
  accelLogDumpCSV();
}

void accelLogDumpCSV() {
  if (accel_log_count == 0) {
    DEBUG_SERIAL.println(F("No data to dump."));
    return;
  }

  DEBUG_SERIAL.println(F("================================================"));
  DEBUG_SERIAL.println(F("  CSV DUMP — COPY FROM HEADER TO END LINE      "));
  DEBUG_SERIAL.println(F("================================================"));
  DEBUG_SERIAL.println();

  // Print calibration info as comments
  DEBUG_SERIAL.println(F("# Calibration offsets applied:"));
  DEBUG_SERIAL.print(F("# AccX_offset=")); DEBUG_SERIAL.print(AccX_offset, 6);
  DEBUG_SERIAL.print(F(" AccY_offset=")); DEBUG_SERIAL.print(AccY_offset, 6);
  DEBUG_SERIAL.print(F(" AccZ_offset=")); DEBUG_SERIAL.println(AccZ_offset, 6);
  DEBUG_SERIAL.print(F("# RateRoll_offset=")); DEBUG_SERIAL.print(RateRoll_offset, 6);
  DEBUG_SERIAL.print(F(" RatePitch_offset=")); DEBUG_SERIAL.print(RatePitch_offset, 6);
  DEBUG_SERIAL.print(F(" RateYaw_offset=")); DEBUG_SERIAL.println(RateYaw_offset, 6);
  DEBUG_SERIAL.println();

  // CSV Header
  DEBUG_SERIAL.println(
    F("Time_s,"
      "AccX_g,AccY_g,AccZ_g,"
      "RateRoll_dps,RatePitch_dps,RateYaw_dps,"
      "AngleRoll_deg,AnglePitch_deg")
  );

  // Data rows
  for (uint16_t i = 0; i < accel_log_count; i++) {
    const AccelSample &s = accel_log[i];

    float t          = s.time_ms_div10  * 0.01f;
    float accX       = s.accX_raw       * 0.001f;
    float accY       = s.accY_raw       * 0.001f;
    float accZ       = s.accZ_raw       * 0.001f;
    float rateRoll   = s.rateRoll_raw   * 0.1f;
    float ratePitch  = s.ratePitch_raw  * 0.1f;
    float rateYaw    = s.rateYaw_raw    * 0.1f;
    float angleRoll  = s.angleRoll_raw  * 0.1f;
    float anglePitch = s.anglePitch_raw * 0.1f;

    DEBUG_SERIAL.print(t, 2);          DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(accX, 4);       DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(accY, 4);       DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(accZ, 4);       DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(rateRoll, 2);   DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(ratePitch, 2);  DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(rateYaw, 2);    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(angleRoll, 2);  DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.println(anglePitch, 2);

    // Prevent serial buffer overflow
    if (i % 50 == 49) delay(10);
  }

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("================================================"));
  DEBUG_SERIAL.println(F("  END OF CSV DUMP                              "));
  DEBUG_SERIAL.println(F("================================================"));
  DEBUG_SERIAL.print(F("  Total samples: ")); DEBUG_SERIAL.println(accel_log_count);
  DEBUG_SERIAL.print(F("  Duration: "));
  DEBUG_SERIAL.print(accel_log[accel_log_count-1].time_ms_div10 * 0.01f, 1);
  DEBUG_SERIAL.println(F("s"));
  DEBUG_SERIAL.println(F("  Type 'l' to start a new log."));
}

// ================================================================
// LIVE PRINT
// ================================================================
void printLive() {
  readIMU();
  DEBUG_SERIAL.println(F("── Live IMU Reading ─────────────────────────────"));
  DEBUG_SERIAL.print(F("  AccX:       ")); DEBUG_SERIAL.print(AccX, 4);
  DEBUG_SERIAL.println(F("g"));
  DEBUG_SERIAL.print(F("  AccY:       ")); DEBUG_SERIAL.print(AccY, 4);
  DEBUG_SERIAL.println(F("g"));
  DEBUG_SERIAL.print(F("  AccZ:       ")); DEBUG_SERIAL.print(AccZ, 4);
  DEBUG_SERIAL.println(F("g"));
  DEBUG_SERIAL.print(F("  RateRoll:   ")); DEBUG_SERIAL.print(RateRoll, 4);
  DEBUG_SERIAL.println(F(" deg/s"));
  DEBUG_SERIAL.print(F("  RatePitch:  ")); DEBUG_SERIAL.print(RatePitch, 4);
  DEBUG_SERIAL.println(F(" deg/s"));
  DEBUG_SERIAL.print(F("  RateYaw:    ")); DEBUG_SERIAL.print(RateYaw, 4);
  DEBUG_SERIAL.println(F(" deg/s"));
  DEBUG_SERIAL.print(F("  AngleRoll:  ")); DEBUG_SERIAL.print(AngleRoll, 4);
  DEBUG_SERIAL.println(F(" deg"));
  DEBUG_SERIAL.print(F("  AnglePitch: ")); DEBUG_SERIAL.print(AnglePitch, 4);
  DEBUG_SERIAL.println(F(" deg"));
  DEBUG_SERIAL.print(F("  Calibrated: "));
  DEBUG_SERIAL.println(calibrated ? F("YES") : F("NO — run 'c' first"));
  DEBUG_SERIAL.println(F("─────────────────────────────────────────────────"));
}

void showHelp() {
  DEBUG_SERIAL.println(F("================================================"));
  DEBUG_SERIAL.println(F("  IMU CALIBRATION & LOGGER — COMMANDS          "));
  DEBUG_SERIAL.println(F("================================================"));
  DEBUG_SERIAL.println(F("  c = Calibrate (keep robot still)"));
  DEBUG_SERIAL.println(F("  l = Start 60s log → auto-dumps CSV"));
  DEBUG_SERIAL.println(F("  q = Stop log early and dump now"));
  DEBUG_SERIAL.println(F("  p = Print live IMU reading"));
  DEBUG_SERIAL.println(F("  r = Reset calibration to zero"));
  DEBUG_SERIAL.println(F("  h = Show this help menu"));
  DEBUG_SERIAL.println(F("================================================"));
  DEBUG_SERIAL.print(F("  Calibrated: "));
  DEBUG_SERIAL.println(calibrated ? F("YES") : F("NO"));
  DEBUG_SERIAL.print(F("  Log active: "));
  DEBUG_SERIAL.println(accel_log_active ? F("YES") : F("NO"));
  DEBUG_SERIAL.println(F("================================================"));
}

// ================================================================
// SERIAL COMMAND HANDLER
// ================================================================
void handleCommands() {
  while (DEBUG_SERIAL.available()) {
    char c = DEBUG_SERIAL.read();
    if (c == '\n' || c == '\r') continue;

    switch (c) {
      case 'c': case 'C':
        runCalibration();
        break;

      case 'l': case 'L':
        accelLogStart();
        break;

      case 'q': case 'Q':
        if (accel_log_active) {
          accelLogStopAndDump();
        } else {
          DEBUG_SERIAL.println(F("No log active. Type 'l' to start."));
        }
        break;

      case 'p': case 'P':
        printLive();
        break;

      case 'r': case 'R':
        AccX_offset = 0.0f;
        AccY_offset = 0.0f;
        AccZ_offset = 0.0f;
        RateRoll_offset  = 0.0f;
        RatePitch_offset = 0.0f;
        RateYaw_offset   = 0.0f;
        calibrated = false;
        DEBUG_SERIAL.println(F("Calibration reset to zero."));
        break;

      case 'h': case 'H':
        showHelp();
        break;

      default:
        DEBUG_SERIAL.print(F("Unknown command: "));
        DEBUG_SERIAL.println(c);
        DEBUG_SERIAL.println(F("Type 'h' for help."));
        break;
    }
  }
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  DEBUG_SERIAL.begin(DEBUG_BAUD);
  delay(1000);

  DEBUG_SERIAL.println(F("================================================"));
  DEBUG_SERIAL.println(F("  MPU-6050 ACCELEROMETER CALIBRATION TOOL      "));
  DEBUG_SERIAL.println(F("  Arduino Mega R3                               "));
  DEBUG_SERIAL.println(F("================================================"));

  initMPU6050();
  showHelp();

  // Take initial reading
  DEBUG_SERIAL.println(F("Initial raw readings (uncalibrated):"));
  printLive();
}

// ================================================================
// LOOP
// ================================================================
void loop() {
  handleCommands();

  // Keep reading IMU continuously
  readIMU();

  // Update logger if active
  accelLogUpdate();

  delay(5);  // ~200Hz read rate — logger samples at 10Hz internally
}