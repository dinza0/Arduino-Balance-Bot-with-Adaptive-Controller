/*
  SelfBalancingBot_RawProtocol2.ino

  Test/mode framework for 2-wheel self-balancing robot using:
    - Arduino Mega / Elegoo Mega
    - MPU6050 IMU over I2C
    - MX-28AR motors running Dynamixel Protocol 2.0
    - MAX485 / RS-485 module with DE/RE direction pin

  This version intentionally uses the same low-level raw packet style as your
  previously working arm263A_finalcode.ino:
    - Serial1 for the Dynamixel RS-485 bus: Mega TX1=18, RX1=19
    - DXL_DIR_PIN = 2
    - 57600 baud
    - manual Protocol 2.0 packets + CRC

  Current default mode: MODE_STATE_PRINT
    - reads IMU + motor states
    - commands zero wheel speed
    - prints state, command velocity, reference errors, and adaptive errors

  Serial commands:
    p = state print / zero speed
    t = Dynamixel wheel-off-ground velocity step test
    c = fixed inverse-dynamics controller
    a = adaptive inverse-dynamics controller
    x = adaptive off / reset adaptive gains
    r = reset controller states
    s or 0 = stop motors
    z = zero wheel positions + integrators

  IMU diagnostic + complementary-filter control version:
    - Raw MPU6050 accel/gyro values are still printed for debugging.
    - theta_rad is the body tilt angle used by the controller.
    - theta_rad is computed from the MPU6050 roll axis using a complementary
      filter: gyro integration for fast motion + accel angle for drift correction.
    - theta_dot_rad_s is the roll gyro rate.
    - phi_rad remains the average wheel rotation angle, not the tilt angle.
*/

#include <Wire.h>
#include <math.h>

// ================================================================
// USER CONFIGURATION
// ================================================================

// Serial ports
#define DEBUG_SERIAL Serial
#define DXL_SERIAL   Serial1   // Wiring diagram: Mega TX1=18 -> MAX485 DI, RX1=19 <- MAX485 RO

const uint32_t DEBUG_BAUD = 115200;  // higher debug baud prevents Serial.print from stalling control
const uint32_t DXL_BAUD   = 57600;

// Read timeout per Dynamixel packet section. At 57600 baud, valid replies
// should arrive in a few ms. Short timeouts prevent the whole Arduino sketch
// from feeling frozen when a read fails.
const uint32_t DXL_READ_TIMEOUT_US = 8000;

// Same as your working arm code: MAX485 DE/RE connected to pin 2
const int DXL_DIR_PIN = 2;      // Wiring diagram: Mega D2 -> MAX485 DE and /RE tied together

// Pin setup from current wiring diagram:
//   MPU6050: SDA -> Mega SDA pin 20, SCL -> Mega SCL pin 21, VCC -> 5V, GND -> GND
//   MAX485:  DI  <- Mega TX1 pin 18
//            RO  -> Mega RX1 pin 19
//            DE and /RE tied together <- Mega D2
//            VCC -> 5V, GND -> GND
//   Dynamixels: MAX485 A/B -> AX/MX PowerHub RS-485 A/B, motors daisy chained

// Motor IDs: change these to match your left/right wheel motors
const uint8_t ID_LEFT  = 4;
const uint8_t ID_RIGHT = 3;

// Sign conventions. Verify before controller testing.
// Goal: forward wheel rotation -> positive phi, positive omega_cmd -> drives forward.
const int8_t LEFT_POS_SIGN  = 1;
const int8_t RIGHT_POS_SIGN = 1;
const int8_t LEFT_CMD_SIGN  = 1;
const int8_t RIGHT_CMD_SIGN = 1;

// Control loop timing
const float CONTROL_HZ = 100.0f;
const float TS_DESIRED = 1.0f / CONTROL_HZ;
const uint32_t CONTROL_PERIOD_US = (uint32_t)(1000000.0f / CONTROL_HZ);

uint32_t last_dxl_cmd_ms = 0;
const uint32_t DXL_CMD_PERIOD_MS = 10;  // 15 ms ~= 66 Hz; try 10 ms later only if RS-485 stays stable

// Dynamixel state reads are slow compared with the control loop.
// During balancing the inverse-dynamics controller mainly needs theta/thetadot,
// so the safest first hardware test is to NOT read motor state while actively
// commanding the motors. This avoids read/write collisions that can freeze the bus.
const bool READ_DXL_DURING_BALANCE = false;
const uint32_t DXL_READ_PERIOD_PRINT_MS   = 100;  // 10 Hz in print/diagnostic mode
const uint32_t DXL_READ_PERIOD_CONTROL_MS = 50;   // 20 Hz if READ_DXL_DURING_BALANCE is true
uint32_t last_dxl_read_ms = 0;
uint32_t last_dxl_warning_ms = 0;
uint8_t dxl_read_fail_count = 0;
const uint8_t DXL_READ_FAIL_LIMIT = 3;
bool dxl_bus_ok = true;

// Safety / command limits
const float THETA_TIP_LIMIT_RAD = 40.0f * DEG_TO_RAD;
const float SOFT_CLAMP_LO_RAD  = 25.0f * DEG_TO_RAD;
const float SOFT_CLAMP_HI_RAD  = 40.0f * DEG_TO_RAD;

// MX-28AR no-load speed is around 55 rpm ~= 5.76 rad/s at 12 V.
// Start conservative. Increase only after testing.
const float MAX_CMD_RAD_S       = 5.5f;
const int32_t MAX_DXL_VEL_RAW   = 230;    // 230*0.229 rpm = 52.7 rpm ~= 5.52 rad/s

// Command slew-rate limiting
const float MAX_CMD_SLEW_RAD_S2 = 500.0f;

// Dynamixel velocity mode setup
const uint32_t DXL_VELOCITY_LIMIT_RAW = 240;  // raw unit 0.229 rpm/count
const uint32_t DXL_PROFILE_ACCEL_RAW  = 0;    // 0 = no velocity profile acceleration limiting

// IMU angle processing
// The MPU6050 hardware DLPF is configured in initMPU6050().
// Raw IMU values are printed, but the controller uses theta_rad from this
// complementary filter so accel-only spikes do not immediately trip safety.
const float IMU_ALPHA = 0.98f;

// Serial plotting/logging
// Serial plotting/logging
const uint32_t PRINT_PERIOD_MS = 200;
const uint32_t CSV_LOG_PERIOD_MS = 20;   // 20 Hz CSV logging, good for plots

bool raw_imu_print_enabled = false;
bool csv_log_enabled = false;
bool csv_header_printed = false;

float theta_offset_rad = 0.0f;

// Choose run mode here.
// MODE_STATE_PRINT is safest. Motors receive zero velocity.
enum RunMode {
  MODE_STATE_PRINT = 0,
  MODE_DXL_STEP_TEST = 1,
  MODE_FIXED_CONTROLLER = 2,
  MODE_ADAPTIVE_CONTROLLER = 3
};
RunMode run_mode = MODE_STATE_PRINT;

// ================================================================
// MX-28AR Protocol 2.0 Control Table Addresses
// ================================================================
const uint16_t ADDR_OPERATING_MODE       = 11;   // 1 byte, 1 = Velocity Control Mode
const uint16_t ADDR_VELOCITY_LIMIT       = 44;   // 4 bytes
const uint16_t ADDR_TORQUE_ENABLE        = 64;   // 1 byte
const uint16_t ADDR_STATUS_RETURN_LEVEL  = 68;   // 1 byte, 1 = reply only to PING/READ; avoids WRITE-response bus collisions
const uint16_t ADDR_BUS_WATCHDOG         = 98;   // 1 byte, not used initially
const uint16_t ADDR_GOAL_VELOCITY        = 104;  // 4 bytes, signed
const uint16_t ADDR_PROFILE_ACCELERATION = 108;  // 4 bytes
const uint16_t ADDR_PRESENT_VELOCITY     = 128;  // 4 bytes, signed
const uint16_t ADDR_PRESENT_POSITION     = 132;  // 4 bytes, signed
const uint16_t ADDR_PRESENT_TEMPERATURE  = 146;  // 1 byte

// Unit conversions
const float DXL_POS_RAD_PER_COUNT   = 0.088f * DEG_TO_RAD;             // Present Position
const float DXL_VEL_RAD_S_PER_COUNT = 0.229f * (2.0f * PI) / 60.0f;    // Goal/Present Velocity

// ================================================================
// MPU6050 variables from your working IMU code
// ================================================================
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

// Raw MPU6050 diagnostic values. These are printed so you can verify
// whether the IMU itself is changing before calibration/filtering.
int16_t RawAccXLSB = 0;
int16_t RawAccYLSB = 0;
int16_t RawAccZLSB = 0;
int16_t RawGyroXLSB = 0;
int16_t RawGyroYLSB = 0;
int16_t RawGyroZLSB = 0;
bool imu_accel_read_ok = false;
bool imu_gyro_read_ok = false;
uint32_t imu_last_read_ms = 0;

// Body tilt states in radians.
// theta_rad is the robot body tilt angle from the IMU roll axis.
// theta_dot_rad_s is the robot body tilt rate from the IMU roll gyro.
// phi_rad below is the wheel angle and is NOT used as the body tilt angle.
float theta_rad = 0.0f;        // filtered body tilt angle used by controller
float theta_dot_rad_s = 0.0f;  // direct gyro roll rate used by controller
float theta_accel_rad = 0.0f;  // raw accel-derived body angle for diagnostics
bool imu_filter_initialized = false;

// ================================================================
// Robot/controller states
// phi_rad = average wheel angle. It is NOT the tilt angle.
// theta_rad = IMU-measured body tilt angle used by the controller.
// Old 5-state variables are still kept for logging/backward compatibility.
// ================================================================
float phi_rad = 0.0f;
float phi_dot_rad_s = 0.0f;
float z_phi = 0.0f;

// Raw motor zero offsets
int32_t pos_left_zero = 0;
int32_t pos_right_zero = 0;

// References
float phi_ref = 0.0f;
float theta_ref = 0.0f;

// Balance trim.
// Positive/negative sign depends on your IMU mounting.
// Start with +/- 0.5 deg and tune.
float theta_ref_trim_rad = 1.8f * DEG_TO_RAD;

//theta_ref =
//    k_outer_phi * (phi_ref - phi_rad)
//  + k_outer_phidot * (0.0f - phi_dot_rad_s);
//
//theta_ref = constrain(theta_ref, -5.0f * DEG_TO_RAD, 5.0f * DEG_TO_RAD);

// ================================================================
// CAD-Based Lumped Inverse Dynamics Model - Updated Assv2
// Origin: midpoint of wheel axle
// Coordinate assumption:
//   Z axis = vertical body COM height
//   Y axis = wheel axle / pitch axis
// CAD assembly excludes wheels. Wheels are added below.
//
// CAD mass properties used:
//   Body mass = 1.422 lb
//   Body COM Z = 2.39 in above axle origin
//   CAD Iyy about output coordinate system = 15.69 lb*in^2
// Wheel values unchanged:
//   65 mm diameter wheel => r1 = 0.0325 m
//   total wheel set mass = 45 g, approximated as 23 g each
//   wheel inertia estimated as solid disk: I = 0.5*m*r^2
// ================================================================

// ================================================================
// CAD-Based Lumped Inverse Dynamics Model
// 90 mm wheels, total robot mass = 1.422 lb including wheels
// ================================================================

const float M_body = 0.603008f;              // kg, body excluding wheels
const float z_COM  = 0.060706f;              // m, body COM height above axle

// Inertia scaled from previous CAD Iyy because mass changed
const float I_body_pitch_axle = 0.00388789f; // kg*m^2

const float m_wheel = 0.021000f;             // kg per wheel
const float r1 = 0.045000f;                  // m, 90 mm diameter wheel

// Solid disk estimate: I = 0.5*m*r^2
const float I_wheel = 2.12625e-5f;           // kg*m^2 per wheel

const float M_total = 0.645008f;             // total robot mass

const float K3 = 0.0366062f;                 // M_body*z_COM
const float K4 = 0.00222222f;                // M_body*z_COM^2
const float K5 = 0.00393042f;                // I_body_pitch_axle + 2*I_wheel
const float Kgrav = 0.359107f;               // 9.81*K3

// Same friction / velocity damping term used in the inverse-dynamics controller.
const float f_fr = 1.0f;

//float inv_omega_n = 28.0f;
//float inv_zeta = 1.0f;

//float inv_Kp = inv_omega_n * inv_omega_n; // --> Actually Used    
float inv_Kp = 650.0f; // NOT USED
//float inv_Kd = 2.0f * inv_zeta * inv_omega_n;   // NOT USED
float inv_Kd = 35.0f; //--> Actually Used
float inv_Ki = 0.0f;  // keep integral off for first hardware tests

float theta_integral = 0.0f;
const float theta_integral_limit = 3.0f;

const float k_tau = 0.025f;  // empirical force/torque-to-speed mapping; lower = stronger wheel-speed command

const int8_t INV_CONTROL_SIGN = -1;  // based on last working fixed-controller recovery direction; flip if both wheels recover wrong
// ================================================================
// Inverse-dynamics adaptive framework
// The adaptive controller uses the fixed controller as the baseline.
// Do not manually tune these to replace the fixed gains.
// They are synced from inv_Kp and inv_Kd when adaptive mode starts.
// ================================================================

float adapt_Kp_base = 0.0f;
float adapt_Kd_base = 0.0f;

float dKp_inv = 0.0f;
float dKd_inv = 0.0f;

// Start conservative. Increase later only after adaptive mode behaves safely.
float gamma_Kp_inv = 300.0f;
float gamma_Kd_inv = 15.0f;

// Leakage pulls adaptive corrections back toward zero.
float lambda_Kp_inv = 0.00f;
float lambda_Kd_inv = 0.00f;

// Adaptive correction limits as fraction of fixed gains.
const float ADAPT_KP_FRAC_LIMIT = 0.25f;   // +/-25% of fixed Kp
const float ADAPT_KD_FRAC_LIMIT = 0.25f;   // +/-25% of fixed Kd

// Reference model Lyapunov Q weights.
// These generate sigma automatically from current baseline Kp/Kd.
const float ADAPT_Q_THETA = 10.0f;
const float ADAPT_Q_THETADOT = 1.0f;

// Adaptation gates.
const float ADAPT_ACTIVE_ANGLE_RAD = 8.0f * DEG_TO_RAD;
const float ADAPT_CMD_SAT_FRAC = 0.85f;
const float ADAPT_DEADZONE_THETA_RAD = 0.0010f;      // about 0.29 deg
const float ADAPT_DEADZONE_RATE_RAD_S = 0.010f;
const float ADAPT_NORM_FLOOR = 0.05f;

float theta_m = 0.0f;
float theta_dot_m = 0.0f;

float dbg_sigma = 0.0f;
float dbg_dKp_dot = 0.0f;
float dbg_dKd_dot = 0.0f;
int dbg_adapt_gate = 0;
// 0 = updating
// 1 = adaptive disabled
// 2 = unsafe/gated
// 3 = deadzone


// Controller safety / mode flags
bool controller_armed = false;
bool adaptive_enabled = false;

// Active safety pause/re-arm logic.
// If the bot tips outside THETA_TIP_LIMIT_RAD for several consecutive samples,
// torque is turned off but the selected controller mode is preserved.
// When you move the bot back inside THETA_REARM_LIMIT_RAD for several samples,
// torque automatically turns back on and the same mode resumes.
bool controller_safety_paused = false;
int tip_bad_count = 0;
int rearm_good_count = 0;
const int TIP_BAD_COUNT_LIMIT = 5;       // 5 samples at 100 Hz ~= 50 ms
const int REARM_GOOD_COUNT_LIMIT = 20;   // 20 samples at 100 Hz ~= 200 ms
const float THETA_REARM_LIMIT_RAD = 15.0f * DEG_TO_RAD;

// Command memory for slew-rate limiting
float last_omega_cmd = 0.0f;

// Timing
uint32_t last_control_us = 0;
uint32_t last_print_ms = 0;

float x_dot = 0.0f;
const float XDOT_LPF_ALPHA = 0.2f;

// When Dynamixel reads are disabled during balance, estimate chassis velocity
// from the last sent wheel command instead of forcing x_dot=0.
// This restores the inverse-dynamics xdot/friction terms without RS-485 read collisions.
float x_dot_est = 0.0f;
float last_sent_omega_cmd = 0.0f;
const float XDOT_CMD_ALPHA = 0.25f;
const float XDOT_SIGN = -1.0f;  // flip to -1 if adding this estimate makes balance worse


// ================================================================
// Protocol 2.0 CRC function from your working code
// ================================================================
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
  unsigned short i, j;
  static const unsigned short crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
  };

  for (j = 0; j < data_blk_size; j++) {
    i = (unsigned short)((crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
  return crc_accum;
}

// ================================================================
// Low-level Dynamixel Protocol 2.0 helpers
// ================================================================

void dxlSetTX() {
  digitalWrite(DXL_DIR_PIN, HIGH);
}

void dxlSetRX() {
  digitalWrite(DXL_DIR_PIN, LOW);
}

void dxlClearRxBuffer() {
  while (DXL_SERIAL.available()) {
    DXL_SERIAL.read();
  }
}

void i32ToLE(int32_t value, uint8_t *out) {
  uint32_t v = (uint32_t)value;
  out[0] = (uint8_t)(v & 0xFF);
  out[1] = (uint8_t)((v >> 8) & 0xFF);
  out[2] = (uint8_t)((v >> 16) & 0xFF);
  out[3] = (uint8_t)((v >> 24) & 0xFF);
}

int32_t leToI32(const uint8_t *p) {
  uint32_t v = ((uint32_t)p[0]) |
               ((uint32_t)p[1] << 8) |
               ((uint32_t)p[2] << 16) |
               ((uint32_t)p[3] << 24);
  return (int32_t)v;
}

uint16_t leToU16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

bool dxlWriteBytes(uint8_t id, uint16_t address, const uint8_t *data, uint16_t data_len) {
  if (data_len > 16) return false;

  uint8_t packet[32];
  uint16_t idx = 0;

  packet[idx++] = 0xFF;
  packet[idx++] = 0xFF;
  packet[idx++] = 0xFD;
  packet[idx++] = 0x00;
  packet[idx++] = id;

  uint16_t length = data_len + 5; // instruction + address(2) + data + CRC(2)
  packet[idx++] = (uint8_t)(length & 0xFF);
  packet[idx++] = (uint8_t)((length >> 8) & 0xFF);
  packet[idx++] = 0x03; // WRITE instruction
  packet[idx++] = (uint8_t)(address & 0xFF);
  packet[idx++] = (uint8_t)((address >> 8) & 0xFF);

  for (uint16_t i = 0; i < data_len; i++) {
    packet[idx++] = data[i];
  }

  unsigned short crc = update_crc(0, packet, idx);
  packet[idx++] = (uint8_t)(crc & 0xFF);
  packet[idx++] = (uint8_t)((crc >> 8) & 0xFF);

  dxlClearRxBuffer();
  dxlSetTX();
  DXL_SERIAL.write(packet, idx);
  DXL_SERIAL.flush();
  dxlSetRX();
  delayMicroseconds(150);

  return true;
}

bool dxlWrite1(uint8_t id, uint16_t address, uint8_t value) {
  return dxlWriteBytes(id, address, &value, 1);
}

bool dxlWrite4(uint8_t id, uint16_t address, int32_t value) {
  uint8_t data[4];
  i32ToLE(value, data);
  return dxlWriteBytes(id, address, data, 4);
}

bool readExactBytes(uint8_t *buffer, uint16_t n, uint32_t timeout_us) {
  uint16_t count = 0;
  uint32_t start = micros();
  while (count < n) {
    if (DXL_SERIAL.available()) {
      buffer[count++] = (uint8_t)DXL_SERIAL.read();
    }
    if ((uint32_t)(micros() - start) > timeout_us) {
      return false;
    }
  }
  return true;
}

bool dxlReadBytes(uint8_t id, uint16_t address, uint16_t read_len, uint8_t *out, uint16_t out_max) {
  if (read_len > out_max || read_len > 32) return false;

  // Build READ instruction packet
  uint8_t packet[16];
  uint16_t idx = 0;
  packet[idx++] = 0xFF;
  packet[idx++] = 0xFF;
  packet[idx++] = 0xFD;
  packet[idx++] = 0x00;
  packet[idx++] = id;
  packet[idx++] = 0x07; // length low = instruction + params(4) + CRC(2)
  packet[idx++] = 0x00;
  packet[idx++] = 0x02; // READ instruction
  packet[idx++] = (uint8_t)(address & 0xFF);
  packet[idx++] = (uint8_t)((address >> 8) & 0xFF);
  packet[idx++] = (uint8_t)(read_len & 0xFF);
  packet[idx++] = (uint8_t)((read_len >> 8) & 0xFF);
  unsigned short crc = update_crc(0, packet, idx);
  packet[idx++] = (uint8_t)(crc & 0xFF);
  packet[idx++] = (uint8_t)((crc >> 8) & 0xFF);

  dxlClearRxBuffer();
  dxlSetTX();
  DXL_SERIAL.write(packet, idx);
  DXL_SERIAL.flush();
  dxlSetRX();
  delayMicroseconds(200);

  // Parse status packet header: FF FF FD 00
  uint8_t header[7];
  uint8_t b;
  uint8_t state = 0;
  uint32_t start = micros();
  while (state < 4) {
    if (DXL_SERIAL.available()) {
      b = (uint8_t)DXL_SERIAL.read();
      if (state == 0 && b == 0xFF) { header[state++] = b; }
      else if (state == 1 && b == 0xFF) { header[state++] = b; }
      else if (state == 2 && b == 0xFD) { header[state++] = b; }
      else if (state == 3 && b == 0x00) { header[state++] = b; }
      else { state = 0; }
    }
    if ((uint32_t)(micros() - start) > DXL_READ_TIMEOUT_US) {
      return false;
    }
  }

  // Read ID and length
  if (!readExactBytes(&header[4], 3, DXL_READ_TIMEOUT_US)) return false;

  uint8_t status_id = header[4];
  uint16_t length = (uint16_t)header[5] | ((uint16_t)header[6] << 8);
  if (status_id != id || length < 4 || length > 40) {
    return false;
  }

  uint8_t rest[48];
  if (!readExactBytes(rest, length, DXL_READ_TIMEOUT_US)) return false;

  // Verify CRC
  uint8_t full[64];
  for (uint8_t i = 0; i < 7; i++) full[i] = header[i];
  for (uint16_t i = 0; i < length; i++) full[7 + i] = rest[i];
  uint16_t total_len = 7 + length;
  uint16_t received_crc = (uint16_t)full[total_len - 2] | ((uint16_t)full[total_len - 1] << 8);
  uint16_t calc_crc = update_crc(0, full, total_len - 2);
  if (received_crc != calc_crc) {
    return false;
  }

  // Status packet: instruction 0x55, error byte, params..., CRC
  if (rest[0] != 0x55) return false;
  uint8_t error = rest[1];
  if (error != 0) return false;

  uint16_t param_len = length - 4;
  if (param_len < read_len) return false;

  for (uint16_t i = 0; i < read_len; i++) {
    out[i] = rest[2 + i];
  }

  return true;
}

// ================================================================
// Dynamixel setup and state functions
// ================================================================

int32_t radpsToDxlVelocityRaw(float omega_rad_s) {
  float raw_f = omega_rad_s / DXL_VEL_RAD_S_PER_COUNT;
  if (raw_f >  MAX_DXL_VEL_RAW) raw_f =  MAX_DXL_VEL_RAW;
  if (raw_f < -MAX_DXL_VEL_RAW) raw_f = -MAX_DXL_VEL_RAW;
  return (int32_t)lround(raw_f);
}

float dxlVelocityRawToRadps(int32_t raw) {
  return (float)raw * DXL_VEL_RAD_S_PER_COUNT;
}

float dxlPositionRawToRad(int32_t raw_delta) {
  return (float)raw_delta * DXL_POS_RAD_PER_COUNT;
}

bool configureOneDynamixel(uint8_t id) {
  bool ok = true;

  // Torque must be off before changing Operating Mode / limits.
  ok &= dxlWrite1(id, ADDR_TORQUE_ENABLE, 0);
  delay(20);

  // Velocity Control Mode = 1.
  ok &= dxlWrite1(id, ADDR_OPERATING_MODE, 1);
  delay(20);

  // Critical for stability on a half-duplex RS-485 bus:
  // do not let WRITE instructions generate status packets.
  // READ instructions still return data, so p/z mode can still read positions.
  ok &= dxlWrite1(id, ADDR_STATUS_RETURN_LEVEL, 1);
  delay(20);

  ok &= dxlWrite4(id, ADDR_VELOCITY_LIMIT, (int32_t)DXL_VELOCITY_LIMIT_RAW);
  delay(20);
  ok &= dxlWrite4(id, ADDR_PROFILE_ACCELERATION, (int32_t)DXL_PROFILE_ACCEL_RAW);
  delay(20);
  ok &= dxlWrite4(id, ADDR_GOAL_VELOCITY, 0);
  delay(20);

  // Keep Bus Watchdog disabled at first; we handle stop in software.
  ok &= dxlWrite1(id, ADDR_BUS_WATCHDOG, 0);
  delay(20);

  ok &= dxlWrite1(id, ADDR_TORQUE_ENABLE, 1);
  delay(20);

  DEBUG_SERIAL.print(F("DXL ID "));
  DEBUG_SERIAL.print(id);
  DEBUG_SERIAL.print(F(" velocity-mode config "));
  DEBUG_SERIAL.println(ok ? F("sent") : F("failed to send"));
  return ok;
}

void initDynamixels() {
  DXL_SERIAL.begin(DXL_BAUD);
  pinMode(DXL_DIR_PIN, OUTPUT);
  dxlSetRX();
  delay(100);

  configureOneDynamixel(ID_LEFT);
  configureOneDynamixel(ID_RIGHT);

  delay(100);

  int32_t vel_dummy;
  if (readDxlState(ID_LEFT, pos_left_zero, vel_dummy)) {
    DEBUG_SERIAL.print(F("Left zero pos = "));
    DEBUG_SERIAL.println(pos_left_zero);
  } else {
    DEBUG_SERIAL.println(F("WARNING: Could not read left motor state."));
  }

  if (readDxlState(ID_RIGHT, pos_right_zero, vel_dummy)) {
    DEBUG_SERIAL.print(F("Right zero pos = "));
    DEBUG_SERIAL.println(pos_right_zero);
  } else {
    DEBUG_SERIAL.println(F("WARNING: Could not read right motor state."));
  }
}

bool readDxlState(uint8_t id, int32_t &present_position, int32_t &present_velocity) {
  // Read Present Velocity(128) and Present Position(132) together: 8 contiguous bytes.
  uint8_t data[8];
  bool ok = dxlReadBytes(id, ADDR_PRESENT_VELOCITY, 8, data, sizeof(data));
  if (!ok) return false;

  present_velocity = leToI32(&data[0]);
  present_position = leToI32(&data[4]);
  return true;
}

uint8_t readDxlTemperature(uint8_t id) {
  uint8_t temp = 0;
  dxlReadBytes(id, ADDR_PRESENT_TEMPERATURE, 1, &temp, 1);
  return temp;
}

void commandWheelVelocityRadps(float omega_cmd_rad_s) {
  omega_cmd_rad_s = constrain(omega_cmd_rad_s, -MAX_CMD_RAD_S, MAX_CMD_RAD_S);

  int32_t raw_common = radpsToDxlVelocityRaw(omega_cmd_rad_s);
  int32_t raw_left   = LEFT_CMD_SIGN  * raw_common;
  int32_t raw_right  = RIGHT_CMD_SIGN * raw_common;

  dxlWrite4(ID_LEFT, ADDR_GOAL_VELOCITY, raw_left);
  delayMicroseconds(250);
  dxlWrite4(ID_RIGHT, ADDR_GOAL_VELOCITY, raw_right);

  // Do not print here during balancing; serial printing inside the command path
  // slows the loop and makes the motor reaction feel delayed.
}

void stopMotors() {
  dxlWrite4(ID_LEFT, ADDR_GOAL_VELOCITY, 0);
  dxlWrite4(ID_RIGHT, ADDR_GOAL_VELOCITY, 0);
  last_dxl_cmd_ms = millis();
  last_sent_omega_cmd = 0.0f;
  x_dot_est = 0.0f;
  x_dot = 0.0f;
}

void torqueOnMotors() {
  dxlWrite1(ID_LEFT,  ADDR_TORQUE_ENABLE, 1);
  delay(20);
  dxlWrite1(ID_RIGHT, ADDR_TORQUE_ENABLE, 1);
  delay(20);
  last_dxl_cmd_ms = 0;
}

void torqueOffMotors() {
  stopMotors();
  delay(20);
  dxlWrite1(ID_LEFT,  ADDR_TORQUE_ENABLE, 0);
  delay(20);
  dxlWrite1(ID_RIGHT, ADDR_TORQUE_ENABLE, 0);
  delay(20);
}


void recoverDxlBus() {
  // Recovery for occasional RS-485 read collisions. This does not reconfigure
  // the motors; it just clears the Arduino serial side and returns to RX.
  dxlSetRX();
  while (DXL_SERIAL.available()) {
    DXL_SERIAL.read();
  }
  DXL_SERIAL.end();
  delay(5);
  DXL_SERIAL.begin(DXL_BAUD);
  dxlSetRX();
  delay(5);
  dxl_read_fail_count = 0;
  dxl_bus_ok = false;
}

void sendWheelCommandRateLimited(float omega_cmd_rad_s, bool force_send) {
  uint32_t now_ms = millis();
  if (force_send || (uint32_t)(now_ms - last_dxl_cmd_ms) >= DXL_CMD_PERIOD_MS) {
    last_dxl_cmd_ms = now_ms;
    commandWheelVelocityRadps(omega_cmd_rad_s);

    // Save the actual final signed command that was sent.
    // Used for command-based x_dot estimate when READ_DXL_DURING_BALANCE=false.
    last_sent_omega_cmd = omega_cmd_rad_s;
  }
}

float tiltSoftScale(float th_rad) {
  float a = fabs(th_rad);
  if (a <= SOFT_CLAMP_LO_RAD) return 1.0f;
  if (a >= SOFT_CLAMP_HI_RAD) return 0.0f;
  return (SOFT_CLAMP_HI_RAD - a) / (SOFT_CLAMP_HI_RAD - SOFT_CLAMP_LO_RAD);
}

bool updateWheelStates(bool force_read) {
  uint32_t now_ms = millis();

  bool balance_mode = (run_mode == MODE_FIXED_CONTROLLER || run_mode == MODE_ADAPTIVE_CONTROLLER);

  // Avoid Dynamixel read/write bus collisions during balancing unless explicitly enabled.
  // The inverse-dynamics controller can run with x_dot = 0 for first tests.
  if (balance_mode && !READ_DXL_DURING_BALANCE && !force_read) {
    phi_dot_rad_s = 0.0f;
    return true;
  }

  uint32_t read_period = balance_mode ? DXL_READ_PERIOD_CONTROL_MS : DXL_READ_PERIOD_PRINT_MS;
  if (!force_read && (uint32_t)(now_ms - last_dxl_read_ms) < read_period) {
    return true;
  }
  last_dxl_read_ms = now_ms;

  int32_t pos_left_raw = 0;
  int32_t pos_right_raw = 0;
  int32_t vel_left_raw = 0;
  int32_t vel_right_raw = 0;

  bool okL = readDxlState(ID_LEFT, pos_left_raw, vel_left_raw);
  delayMicroseconds(250);  // small bus turnaround gap between two status reads
  bool okR = readDxlState(ID_RIGHT, pos_right_raw, vel_right_raw);

  if (!okL || !okR) {
    dxl_read_fail_count++;

    // Do not spam Serial. Printing too much during a bus fault can make the sketch
    // look frozen and delay serial command handling.
    if ((uint32_t)(now_ms - last_dxl_warning_ms) > 500) {
      last_dxl_warning_ms = now_ms;
      DEBUG_SERIAL.print(F("WARNING: Dynamixel state read failed. okL="));
      DEBUG_SERIAL.print(okL);
      DEBUG_SERIAL.print(F(" okR="));
      DEBUG_SERIAL.print(okR);
      DEBUG_SERIAL.print(F(" fail_count="));
      DEBUG_SERIAL.println(dxl_read_fail_count);
    }

    if (dxl_read_fail_count >= DXL_READ_FAIL_LIMIT) {
      recoverDxlBus();
    }

    // Keep the previous phi value and avoid using stale wheel velocity in x_dot.
    phi_dot_rad_s = 0.0f;
    return false;
  }

  dxl_read_fail_count = 0;
  dxl_bus_ok = true;

  float phi_left  = LEFT_POS_SIGN  * dxlPositionRawToRad(pos_left_raw  - pos_left_zero);
  float phi_right = RIGHT_POS_SIGN * dxlPositionRawToRad(pos_right_raw - pos_right_zero);

  float phidot_left  = LEFT_POS_SIGN  * dxlVelocityRawToRadps(vel_left_raw);
  float phidot_right = RIGHT_POS_SIGN * dxlVelocityRawToRadps(vel_right_raw);

  phi_rad = 0.5f * (phi_left + phi_right);
  phi_dot_rad_s = 0.5f * (phidot_left + phidot_right);
  return true;
}

// ================================================================
// ================================================================
// MPU6050 functions - kept aligned with your tested IMU_new.ino
// ================================================================
void resetIMUFilter() {
  // Reinitialize the complementary filter to the current accel angle on
  // the next updateIMU() call.
  imu_filter_initialized = false;
  theta_rad = 0.0f;
  theta_dot_rad_s = 0.0f;
  theta_accel_rad = 0.0f;
}



void gyro_signals(void) {
    imu_accel_read_ok = false;
    imu_gyro_read_ok = false;

    // 1. Request Accelerometer Data (Register 0x3B)
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);

    // Safety check: only process if all 6 bytes are received
    if (Wire.available() >= 6) {
        RawAccXLSB = (int16_t)((Wire.read() << 8) | Wire.read());
        RawAccYLSB = (int16_t)((Wire.read() << 8) | Wire.read());
        RawAccZLSB = (int16_t)((Wire.read() << 8) | Wire.read());
        imu_accel_read_ok = true;

        // Convert to Gs. MPU6050 accel range is +/-8g, so sensitivity is 4096 LSB/g.
        // These include your current empirical accel offsets.
        //AccX = (float)RawAccXLSB / 4096.0f + 0.04f;
        //AccY = (float)RawAccYLSB / 4096.0f - 0.001f;
        //AccZ = (float)RawAccZLSB / 4096.0f - 0.02f;
        //AccX = (float)RawAccXLSB / 4096.0f - 0.03616;
        //AccY = (float)RawAccYLSB / 4096.0f + 0.036813;
        //AccZ = (float)RawAccZLSB / 4096.0f - 0.000948;
        AccX = (float)RawAccXLSB / 4096.0f + 0.028060;
        AccY = (float)RawAccYLSB / 4096.0f + 0.000721;
        AccZ = (float)RawAccZLSB / 4096.0f - 0.017254;

    } else {
        // Flush any partial bytes so the next read starts cleanly.
        while (Wire.available()) Wire.read();
    }

    // 2. Request Gyroscope Data (Register 0x43)
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);

    if (Wire.available() >= 6) {
        RawGyroXLSB = (int16_t)((Wire.read() << 8) | Wire.read());
        RawGyroYLSB = (int16_t)((Wire.read() << 8) | Wire.read());
        RawGyroZLSB = (int16_t)((Wire.read() << 8) | Wire.read());
        imu_gyro_read_ok = true;

        // Convert to deg/s. MPU6050 gyro range is +/-500 dps, so sensitivity is 65.5 LSB/(deg/s).
        // Calibration offsets are subtracted later in updateIMU().
        RateRoll  = (float)RawGyroXLSB / 65.5f;
        RatePitch = (float)RawGyroYLSB / 65.5f;
        RateYaw   = (float)RawGyroZLSB / 65.5f;
    } else {
        while (Wire.available()) Wire.read();
    }

    // 3. Calculate raw accelerometer tilt angles only when accel read succeeded.
    if (imu_accel_read_ok) {
        // Roll: rotation around X-axis
        
        AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180.0f / PI);

        // Pitch: rotation around Y-axis
        AnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180.0f / PI);
    }

    imu_last_read_ms = millis();
}
void initMPU6050() {
    // This is the MPU setup/calibration sequence from your tested IMU_new.ino.
    // Serial.begin() is intentionally not repeated here because the main sketch
    // initializes DEBUG_SERIAL once in setup().
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH); // LED ON: Calibrating (Keep sensor still!)
    
    Wire.begin();
    Wire.setClock(100000); // 100kHz for high stability on breadboards
    // Prevent I2C lockups from freezing the sketch if motor noise glitches SDA/SCL.
    Wire.setWireTimeout(3000, true);
    delay(250);

    // Wake up MPU-6050
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    // Set Low Pass Filter (Register 0x1A)
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();

    // Configure Accelerometer (+/- 8g)
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    // Configure Gyroscope (500dps)
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    // Run Calibration
    RateCalibrationRoll = 0.0f;
    RateCalibrationPitch = 0.0f;
    RateCalibrationYaw = 0.0f;
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
        gyro_signals();
        RateCalibrationRoll += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw += RateYaw;
        delay(1);
    }
    
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;

    float angleRollSum = 0.0f;

    for (int i = 0; i < 500; i++) {
      gyro_signals();
      angleRollSum += AngleRoll * DEG_TO_RAD;
      delay(2);
    }

    theta_offset_rad = angleRollSum / 500.0f;

    digitalWrite(13, LOW); // LED OFF: Setup complete!
    DEBUG_SERIAL.println("Calibration Finished. Starting Data Stream...");
}

void updateIMU(float dt) {
  if (dt <= 0.0f || dt > 0.05f) dt = TS_DESIRED;

  gyro_signals();

  // Apply gyro calibration offsets.
  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;

  // IMPORTANT AXIS CONVENTION:
  // theta = robot body tilt angle from the IMU.
  // For your current IMU mounting, theta comes from the MPU6050 roll axis.
  // phi = average wheel angle only; phi is NOT the body tilt angle.
  theta_accel_rad = (AngleRoll * DEG_TO_RAD) - theta_offset_rad;
  float gyro_theta_rad_s = RateRoll * DEG_TO_RAD;

  if (!imu_filter_initialized) {
    theta_rad = theta_accel_rad;
    imu_filter_initialized = true;
  }

  // Complementary filter for the control angle:
  //   fast motion from gyro integration
  //   long-term drift correction from accelerometer angle
  theta_rad =
      IMU_ALPHA * (theta_rad + gyro_theta_rad_s * dt)
    + (1.0f - IMU_ALPHA) * theta_accel_rad;

  theta_dot_rad_s = gyro_theta_rad_s;
}


float forceToOmegaCmd(float F_contact) {
  float tau_wheel = F_contact * r1;
  float omega_w = tau_wheel / k_tau;

  omega_w = constrain(omega_w, -MAX_CMD_RAD_S, MAX_CMD_RAD_S);
  return omega_w;
}

float computeInverseDynamicsForce(float th, float th_dot, float th_ddot_cmd, float xdot) {
  float sin_th  = sin(th);
  float cos_th  = cos(th);
  float sin_2th = sin(2.0f * th);
  float cos2_th = cos_th * cos_th;
  float th_dot2 = th_dot * th_dot;

  float num =
      (K5 + K4 * cos2_th) * th_ddot_cmd
    - K3 * th_dot * sin_th
    + K4 * th_dot * cos_th
    - Kgrav * sin_th
    + K3 * xdot * sin_th * th_dot
    - K4 * sin_2th * th_dot2;

  float den = K3 * cos_th;

  if (fabs(den) < 0.001f) {
    den = (den >= 0.0f) ? 0.001f : -0.001f;
  }

  float x_ddot_computed = num / den;

  float F =
      -M_total * x_ddot_computed
    - K3 * sin_th * th_dot2
    + K3 * cos_th * th_ddot_cmd
    + f_fr * xdot;

  return F;
}

// Controller modes
// ================================================================


void resetControllerStates() {
  z_phi = 0.0f;

  last_omega_cmd = 0.0f;

  theta_integral = 0.0f;

  resetInverseAdaptiveStatesOnly();

  x_dot = 0.0f;
  x_dot_est = 0.0f;
  last_sent_omega_cmd = 0.0f;

  tip_bad_count = 0;
  rearm_good_count = 0;
  last_dxl_cmd_ms = 0;
}


void updateIntegralStates(float dt) {
  float e_phi = phi_ref - phi_rad;

  z_phi += e_phi * dt;

  // Anti-windup clamp
  z_phi = constrain(z_phi, -2.0f, 2.0f);
}

float applyCommandSlewLimit(float omega_cmd, float dt) {
  float max_delta = MAX_CMD_SLEW_RAD_S2 * dt;
  float delta = omega_cmd - last_omega_cmd;

  if (delta > max_delta) {
    omega_cmd = last_omega_cmd + max_delta;
  } else if (delta < -max_delta) {
    omega_cmd = last_omega_cmd - max_delta;
  }

  last_omega_cmd = omega_cmd;
  return omega_cmd;
}

float computeFixedController(float dt) {
  
  float theta_ref_eff = theta_ref + theta_ref_trim_rad;

  float e = theta_ref_eff - theta_rad;
  float e_dot = 0.0f - theta_dot_rad_s;

  theta_integral += e * dt;
  theta_integral = constrain(theta_integral, -theta_integral_limit, theta_integral_limit);

  float theta_ddot_cmd =
      inv_Kp * e
    + inv_Kd * e_dot
    + inv_Ki * theta_integral;

  float F_control =
    computeInverseDynamicsForce(theta_rad, theta_dot_rad_s, theta_ddot_cmd, x_dot);

  float omega_cmd = forceToOmegaCmd(F_control);

  omega_cmd *= tiltSoftScale(theta_rad);
  omega_cmd = INV_CONTROL_SIGN * omega_cmd;
  omega_cmd = constrain(omega_cmd, -MAX_CMD_RAD_S, MAX_CMD_RAD_S);
  omega_cmd = applyCommandSlewLimit(omega_cmd, dt);

  return omega_cmd;
}


void syncAdaptiveBaselineToFixed() {
  // Adaptive mode starts from the exact currently optimized fixed gains.
  adapt_Kp_base = inv_Kp;
  adapt_Kd_base = inv_Kd;
}

void resetInverseAdaptiveStatesOnly() {
  dKp_inv = 0.0f;
  dKd_inv = 0.0f;

  theta_m = theta_rad;
  theta_dot_m = theta_dot_rad_s;
}

void leakAdaptiveCorrections(float dt) {
  dKp_inv += dt * (-lambda_Kp_inv * dKp_inv);
  dKd_inv += dt * (-lambda_Kd_inv * dKd_inv);
}

void projectAdaptiveCorrections() {
  dKp_inv = constrain(dKp_inv, -ADAPT_KP_FRAC_LIMIT, ADAPT_KP_FRAC_LIMIT);
  dKd_inv = constrain(dKd_inv, -ADAPT_KD_FRAC_LIMIT, ADAPT_KD_FRAC_LIMIT);
}

void getInverseSigmaCoeffs(float &p12, float &p22) {
  float Kp_m = (adapt_Kp_base > 1.0f) ? adapt_Kp_base : inv_Kp;
  float Kd_m = (adapt_Kd_base > 1.0f) ? adapt_Kd_base : inv_Kd;

  p12 = ADAPT_Q_THETA / (2.0f * Kp_m);
  p22 = (2.0f * p12 + ADAPT_Q_THETADOT) / (2.0f * Kd_m);
}

float computeInverseSigma(float em_theta, float em_theta_dot) {
  float p12, p22;
  getInverseSigmaCoeffs(p12, p22);

  return p12 * em_theta + p22 * em_theta_dot;
}

float getThetaRefEff() {
  return theta_ref + theta_ref_trim_rad;
}

void updatePitchReferenceModel(float dt) {
  //float theta_ref_eff = theta_ref + theta_ref_trim_rad;

  if (adapt_Kp_base <= 0.0f || adapt_Kd_base <= 0.0f) {
    syncAdaptiveBaselineToFixed();
  }

  // Match the fixed controller reference exactly.
  float theta_ref_eff = theta_ref + theta_ref_trim_rad;

  float theta_m_ddot =
      adapt_Kp_base * (theta_ref_eff - theta_m)
    + adapt_Kd_base * (0.0f - theta_dot_m);

  theta_dot_m += theta_m_ddot * dt;
  theta_m += theta_dot_m * dt;
}

void updateInverseAdaptiveLaw(float dt) {
  if (!adaptive_enabled) {
    dbg_adapt_gate = 1;
    return;
  }

  if (adapt_Kp_base <= 0.0f || adapt_Kd_base <= 0.0f) {
    syncAdaptiveBaselineToFixed();
  }

  updatePitchReferenceModel(dt);

  float theta_ref_eff = getThetaRefEff();

  float e_theta = theta_ref_eff - theta_rad;
  float e_theta_dot = 0.0f - theta_dot_rad_s;

  float em_theta = theta_rad - theta_m;
  float em_theta_dot = theta_dot_rad_s - theta_dot_m;

  float sigma = computeInverseSigma(em_theta, em_theta_dot);
  dbg_sigma = sigma;

  bool imu_ok = imu_accel_read_ok && imu_gyro_read_ok;
  bool near_upright = fabs(theta_rad) < ADAPT_ACTIVE_ANGLE_RAD;
  bool not_saturated = fabs(last_omega_cmd) < (ADAPT_CMD_SAT_FRAC * MAX_CMD_RAD_S);

  bool safe_to_adapt =
      controller_armed &&
      !controller_safety_paused &&
      imu_ok &&
      near_upright &&
      not_saturated;

  if (!safe_to_adapt) {
    dbg_adapt_gate = 2;
    leakAdaptiveCorrections(dt);
    projectAdaptiveCorrections();
    return;
  }

  bool inside_deadzone =
      fabs(em_theta) < ADAPT_DEADZONE_THETA_RAD &&
      fabs(em_theta_dot) < ADAPT_DEADZONE_RATE_RAD_S;

  if (inside_deadzone) {
    dbg_adapt_gate = 3;
    leakAdaptiveCorrections(dt);
    projectAdaptiveCorrections();
    return;

  }

  float phi_Kp = adapt_Kp_base * e_theta;
  float phi_Kd = adapt_Kd_base * e_theta_dot;

  float norm_factor =
      ADAPT_NORM_FLOOR
    + phi_Kp * phi_Kp
    + phi_Kd * phi_Kd;

  float dKp_dot =
      (-gamma_Kp_inv * sigma * phi_Kp) / norm_factor
    - lambda_Kp_inv * dKp_inv;

  float dKd_dot =
      (-gamma_Kd_inv * sigma * phi_Kd) / norm_factor
    - lambda_Kd_inv * dKd_inv;

  dbg_adapt_gate = 0;
  dbg_dKp_dot = dKp_dot;
  dbg_dKd_dot = dKd_dot;

  dKp_inv += dt * dKp_dot;
  dKd_inv += dt * dKd_dot;

  projectAdaptiveCorrections();
}


float computeAdaptiveController(float dt) {
  if (adapt_Kp_base <= 0.0f || adapt_Kd_base <= 0.0f) {
    syncAdaptiveBaselineToFixed();
  }

  updateInverseAdaptiveLaw(dt);

  float Kp_eff = adapt_Kp_base * (1.0f + dKp_inv);
  float Kd_eff = adapt_Kd_base * (1.0f + dKd_inv);

  float theta_ref_eff = getThetaRefEff();
  float e_theta = theta_ref_eff - theta_rad;
  float e_theta_dot = 0.0f - theta_dot_rad_s;

  theta_integral += e_theta * dt;
  theta_integral = constrain(theta_integral, -theta_integral_limit, theta_integral_limit);

  float theta_ddot_cmd =
      Kp_eff * e_theta
    + Kd_eff * e_theta_dot
    + inv_Ki * theta_integral;

  float F_control =
    computeInverseDynamicsForce(theta_rad, theta_dot_rad_s, theta_ddot_cmd, x_dot);

  float omega_cmd = forceToOmegaCmd(F_control);

  omega_cmd *= tiltSoftScale(theta_rad);
  omega_cmd = INV_CONTROL_SIGN * omega_cmd;

  omega_cmd = constrain(omega_cmd, -MAX_CMD_RAD_S, MAX_CMD_RAD_S);
  omega_cmd = applyCommandSlewLimit(omega_cmd, dt);

  return omega_cmd;
}

float computeDxlStepTestCommand() {
  // Wheel-off-ground test only. Produces square-wave velocity command.
  const float STEP_RAD_S = 4.0f;
  const uint32_t PERIOD_MS = 4000;
  uint32_t phase = (millis() / PERIOD_MS) % 2;
  return (phase == 0) ? STEP_RAD_S : -STEP_RAD_S;
}

bool isControllerMode() {
  return (run_mode == MODE_FIXED_CONTROLLER || run_mode == MODE_ADAPTIVE_CONTROLLER);
}

bool updateActiveSafety() {
  // Safety auto-pause/re-arm only applies to balance controllers.
  // State print and wheel-off-ground step test are not automatically paused.
  if (!isControllerMode()) {
    controller_safety_paused = false;
    tip_bad_count = 0;
    rearm_good_count = 0;
    return true;
  }

  float abs_theta = fabs(theta_rad);

  if (controller_safety_paused) {
    // Stay off while tilted, but keep reading IMU.
    if (abs_theta < THETA_REARM_LIMIT_RAD && imu_accel_read_ok && imu_gyro_read_ok) {
      rearm_good_count++;
    } else {
      rearm_good_count = 0;
    }

    if (rearm_good_count >= REARM_GOOD_COUNT_LIMIT) {
      // Re-arm the same selected mode once the bot is back near upright.
      controller_safety_paused = false;
      tip_bad_count = 0;
      rearm_good_count = 0;
      resetControllerStates();
      resetIMUFilter();
      torqueOnMotors();
      controller_armed = true;
      DEBUG_SERIAL.println(F("SAFETY REARM: back inside safe tilt range, controller resumed."));
      return true;
    }

    return false;
  }

  // Require several consecutive bad samples before pausing, so one bad
  // accel/gyro spike does not shut the controller off.
  if (abs_theta > THETA_TIP_LIMIT_RAD || !imu_accel_read_ok || !imu_gyro_read_ok) {
    tip_bad_count++;
  } else {
    tip_bad_count = 0;
  }

  if (tip_bad_count >= TIP_BAD_COUNT_LIMIT) {
    controller_safety_paused = true;
    controller_armed = false;
    rearm_good_count = 0;
    last_omega_cmd = 0.0f;
    torqueOffMotors();
    DEBUG_SERIAL.println(F("SAFETY PAUSE: tilt outside safe range. Move bot upright to auto-rearm."));
    return false;
  }

  return true;
}

void fullStopToPrintMode() {
  torqueOffMotors();
  resetControllerStates();
  controller_armed = false;
  adaptive_enabled = false;
  controller_safety_paused = false;
  run_mode = MODE_STATE_PRINT;
}

void printCsvHeader() {
  DEBUG_SERIAL.println(F(
    "time_ms,"
    "mode,"
    "theta_ref,"
    "theta,"
    "theta_error,"
    "thetadot,"
    "theta_accel,"
    "omega_cmd,"
    "last_wcmd,"
    "x_dot,"
    "phi,"
    "phidot,"
    "theta_m,"
    "thetadot_m,"
    "em_theta,"
    "em_thetadot,"
    "sigma,"
    "dKp_frac,"
    "dKd_frac,"
    "dKp_pct,"
    "dKd_pct,"
    "Kp_eff,"
    "Kd_eff,"
    "adapt_gate,"
    "safety_pause,"
    "dxl_ok,"
    "imu_ok"
  ));
}

void printCsvLog(float omega_cmd) {
  float theta_ref_eff = getThetaRefEff();

  float theta_error = theta_ref_eff - theta_rad;

  float em_theta = theta_rad - theta_m;
  float em_thetadot = theta_dot_rad_s - theta_dot_m;

  float sigma = computeInverseSigma(em_theta, em_thetadot);

  // If your adaptive code uses fractional corrections:
  float Kp_eff = adapt_Kp_base * (1.0f + dKp_inv);
  float Kd_eff = adapt_Kd_base * (1.0f + dKd_inv);

  // If you are in fixed mode, adapt_Kp_base may be zero.
  // For cleaner plots, use fixed gains when adaptive baseline is not active.
  if (adapt_Kp_base <= 1.0f) Kp_eff = inv_Kp;
  if (adapt_Kd_base <= 1.0f) Kd_eff = inv_Kd;

  if (!csv_header_printed) {
    printCsvHeader();
    csv_header_printed = true;
  }

  DEBUG_SERIAL.print(millis()); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print((int)run_mode); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(theta_ref_eff, 6); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(theta_rad, 6); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(theta_error, 6); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(theta_dot_rad_s, 6); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(theta_accel_rad, 6); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(omega_cmd, 6); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(last_sent_omega_cmd, 6); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(x_dot, 6); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(phi_rad, 6); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(phi_dot_rad_s, 6); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(theta_m, 6); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(theta_dot_m, 6); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(em_theta, 6); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(em_thetadot, 6); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(sigma, 8); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(dKp_inv, 6); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(dKd_inv, 6); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(100.0f * dKp_inv, 3); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(100.0f * dKd_inv, 3); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(Kp_eff, 4); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(Kd_eff, 4); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(dbg_adapt_gate); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(controller_safety_paused); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(dxl_bus_ok); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.println(imu_accel_read_ok && imu_gyro_read_ok);
}

void printState(float omega_cmd) {
  if (csv_log_enabled) {
    printCsvLog(omega_cmd);
    return;
  }
  if (isControllerMode() && !raw_imu_print_enabled) {
    // Compact controller-mode print. The previous raw-IMU line was very long
    // and could block DEBUG_SERIAL long enough to make the controller seem frozen.
    DEBUG_SERIAL.print(F("time_ms=")); DEBUG_SERIAL.print(millis());
    DEBUG_SERIAL.print(F(" mode=")); DEBUG_SERIAL.print((int)run_mode);
    DEBUG_SERIAL.print(F(" omega_cmd=")); DEBUG_SERIAL.print(omega_cmd, 4);
    DEBUG_SERIAL.print(F(" theta=")); DEBUG_SERIAL.print(theta_rad, 4);
    DEBUG_SERIAL.print(F(" thetadot=")); DEBUG_SERIAL.print(theta_dot_rad_s, 4);
    DEBUG_SERIAL.print(F(" theta_accel=")); DEBUG_SERIAL.print(theta_accel_rad, 4);
    DEBUG_SERIAL.print(F(" phi=")); DEBUG_SERIAL.print(phi_rad, 4);
    DEBUG_SERIAL.print(F(" phidot=")); DEBUG_SERIAL.print(phi_dot_rad_s, 4);
    DEBUG_SERIAL.print(F(" x_dot=")); DEBUG_SERIAL.print(x_dot, 4);
    DEBUG_SERIAL.print(F(" last_wcmd=")); DEBUG_SERIAL.print(last_sent_omega_cmd, 4);
    DEBUG_SERIAL.print(F(" imuOK=")); DEBUG_SERIAL.print(imu_accel_read_ok && imu_gyro_read_ok);
    DEBUG_SERIAL.print(F(" armed=")); DEBUG_SERIAL.print(controller_armed);
    DEBUG_SERIAL.print(F(" safety_pause=")); DEBUG_SERIAL.print(controller_safety_paused);
    DEBUG_SERIAL.print(F(" dxl_ok=")); DEBUG_SERIAL.print(dxl_bus_ok);
    DEBUG_SERIAL.print(F(" dKp_frac=")); DEBUG_SERIAL.print(dKp_inv, 6);
    DEBUG_SERIAL.print(F(" dKd_frac=")); DEBUG_SERIAL.print(dKd_inv, 6);
    DEBUG_SERIAL.print(F(" dKp_pct=")); DEBUG_SERIAL.print(100.0f * dKp_inv, 3);
    DEBUG_SERIAL.print(F(" dKd_pct=")); DEBUG_SERIAL.print(100.0f * dKd_inv, 3);
    DEBUG_SERIAL.print(F(" Kp_eff=")); DEBUG_SERIAL.print(adapt_Kp_base * (1.0f + dKp_inv), 2);
    DEBUG_SERIAL.print(F(" Kd_eff=")); DEBUG_SERIAL.print(adapt_Kd_base * (1.0f + dKd_inv), 2);
    DEBUG_SERIAL.print(F(" gate=")); DEBUG_SERIAL.print(dbg_adapt_gate);
    DEBUG_SERIAL.print(F(" sigma=")); DEBUG_SERIAL.print(dbg_sigma, 7);
    DEBUG_SERIAL.print(F(" dKp_dot=")); DEBUG_SERIAL.print(dbg_dKp_dot, 7);
    DEBUG_SERIAL.print(F(" dKd_dot=")); DEBUG_SERIAL.print(dbg_dKd_dot, 7);
    DEBUG_SERIAL.println();
    return;
  }

  printStateVerbose(omega_cmd);
}

void printStateVerbose(float omega_cmd) {
  float e_phi = phi_ref - phi_rad;
  float e_theta = theta_ref - theta_rad;
  

  // Inverse adaptive sigma for plotting/logging only.
  float Kp_m_log = (adapt_Kp_base > 1.0f) ? adapt_Kp_base : inv_Kp;
  float Kd_m_log = (adapt_Kd_base > 1.0f) ? adapt_Kd_base : inv_Kd;
  float p12_log = ADAPT_Q_THETA / (2.0f * Kp_m_log);
  float p22_log = (2.0f * p12_log + ADAPT_Q_THETADOT) / (2.0f * Kd_m_log);
  //float sigma_log = p12_log * (theta_rad - theta_m)
  //                + p22_log * (theta_dot_rad_s - theta_dot_m);
  float em_theta_inv = theta_rad - theta_m;
  float em_thetadot_inv = theta_dot_rad_s - theta_dot_m;
  float sigma_log = computeInverseSigma(em_theta_inv, em_thetadot_inv);

  DEBUG_SERIAL.print(F("time_ms=")); DEBUG_SERIAL.print(millis());
  DEBUG_SERIAL.print(F(" mode=")); DEBUG_SERIAL.print((int)run_mode);
  DEBUG_SERIAL.print(F(" omega_cmd=")); DEBUG_SERIAL.print(omega_cmd, 4);
  DEBUG_SERIAL.print(F(" phi=")); DEBUG_SERIAL.print(phi_rad, 4);
  DEBUG_SERIAL.print(F(" phidot=")); DEBUG_SERIAL.print(phi_dot_rad_s, 4);
  DEBUG_SERIAL.print(F(" theta=")); DEBUG_SERIAL.print(theta_rad, 4);
  DEBUG_SERIAL.print(F(" thetadot=")); DEBUG_SERIAL.print(theta_dot_rad_s, 4);
  DEBUG_SERIAL.print(F(" z_phi=")); DEBUG_SERIAL.print(z_phi, 4);

  // Raw IMU diagnostics plus filtered controller angle.
  // raw_a* and raw_g* are raw MPU6050 register values.
  // acc_* is converted acceleration in g after empirical offsets.
  // rate_* is gyro deg/s after calibration offset subtraction in updateIMU().
  // theta_accel is direct accel angle; theta is complementary-filtered controller angle.
  DEBUG_SERIAL.print(F(" imuAok=")); DEBUG_SERIAL.print(imu_accel_read_ok);
  DEBUG_SERIAL.print(F(" imuGok=")); DEBUG_SERIAL.print(imu_gyro_read_ok);
  DEBUG_SERIAL.print(F(" raw_ax=")); DEBUG_SERIAL.print(RawAccXLSB);
  DEBUG_SERIAL.print(F(" raw_ay=")); DEBUG_SERIAL.print(RawAccYLSB);
  DEBUG_SERIAL.print(F(" raw_az=")); DEBUG_SERIAL.print(RawAccZLSB);
  DEBUG_SERIAL.print(F(" raw_gx=")); DEBUG_SERIAL.print(RawGyroXLSB);
  DEBUG_SERIAL.print(F(" raw_gy=")); DEBUG_SERIAL.print(RawGyroYLSB);
  DEBUG_SERIAL.print(F(" raw_gz=")); DEBUG_SERIAL.print(RawGyroZLSB);
  DEBUG_SERIAL.print(F(" acc_x=")); DEBUG_SERIAL.print(AccX, 4);
  DEBUG_SERIAL.print(F(" acc_y=")); DEBUG_SERIAL.print(AccY, 4);
  DEBUG_SERIAL.print(F(" acc_z=")); DEBUG_SERIAL.print(AccZ, 4);
  DEBUG_SERIAL.print(F(" rate_roll=")); DEBUG_SERIAL.print(RateRoll, 4);
  DEBUG_SERIAL.print(F(" rate_pitch=")); DEBUG_SERIAL.print(RatePitch, 4);
  DEBUG_SERIAL.print(F(" rate_yaw=")); DEBUG_SERIAL.print(RateYaw, 4);
  DEBUG_SERIAL.print(F(" angle_roll_deg=")); DEBUG_SERIAL.print(AngleRoll, 4);
  DEBUG_SERIAL.print(F(" angle_pitch_deg=")); DEBUG_SERIAL.print(AnglePitch, 4);
  DEBUG_SERIAL.print(F(" theta_accel=")); DEBUG_SERIAL.print(theta_accel_rad, 4);
  DEBUG_SERIAL.print(F(" theta_filtered=")); DEBUG_SERIAL.print(theta_rad, 4);
  DEBUG_SERIAL.print(F(" theta_offset=")); DEBUG_SERIAL.print(theta_offset_rad, 4);

  // Reference tracking errors for serial plotting.
  DEBUG_SERIAL.print(F(" e_phi=")); DEBUG_SERIAL.print(e_phi, 4);
  DEBUG_SERIAL.print(F(" e_theta=")); DEBUG_SERIAL.print(e_theta, 4);

  // Model-following errors for adaptive-control debugging.
  DEBUG_SERIAL.print(F(" em_theta=")); DEBUG_SERIAL.print(em_theta_inv, 4);
  DEBUG_SERIAL.print(F(" em_thetadot=")); DEBUG_SERIAL.print(em_thetadot_inv, 4);
  DEBUG_SERIAL.print(F(" theta_m=")); DEBUG_SERIAL.print(theta_m, 4);
  DEBUG_SERIAL.print(F(" thetadot_m=")); DEBUG_SERIAL.print(theta_dot_m, 4);

  // Command velocity and adaptive variables.
  
  DEBUG_SERIAL.print(F(" sigma=")); DEBUG_SERIAL.print(sigma_log, 4);
  DEBUG_SERIAL.print(F(" dKp_inv=")); DEBUG_SERIAL.print(dKp_inv, 7);
  DEBUG_SERIAL.print(F(" dKd_inv=")); DEBUG_SERIAL.print(dKd_inv, 7);
  DEBUG_SERIAL.print(F(" Kp_base=")); DEBUG_SERIAL.print(adapt_Kp_base, 4);
  DEBUG_SERIAL.print(F(" Kd_base=")); DEBUG_SERIAL.print(adapt_Kd_base, 4);
  DEBUG_SERIAL.print(F(" Kp_eff=")); DEBUG_SERIAL.print(adapt_Kp_base + dKp_inv, 4);
  DEBUG_SERIAL.print(F(" Kd_eff=")); DEBUG_SERIAL.print(adapt_Kd_base + dKd_inv, 4);
  DEBUG_SERIAL.print(F(" x_dot=")); DEBUG_SERIAL.print(x_dot, 4);
  DEBUG_SERIAL.print(F(" x_dot_est=")); DEBUG_SERIAL.print(x_dot_est, 4);
  DEBUG_SERIAL.print(F(" last_wcmd=")); DEBUG_SERIAL.print(last_sent_omega_cmd, 4);
  DEBUG_SERIAL.print(F(" armed=")); DEBUG_SERIAL.print(controller_armed);
  DEBUG_SERIAL.print(F(" adapt=")); DEBUG_SERIAL.print(adaptive_enabled);
  DEBUG_SERIAL.print(F(" safety_pause=")); DEBUG_SERIAL.print(controller_safety_paused);
  DEBUG_SERIAL.print(F(" dxl_ok=")); DEBUG_SERIAL.print(dxl_bus_ok);
  DEBUG_SERIAL.print(F(" dxl_fail=")); DEBUG_SERIAL.print(dxl_read_fail_count);
  DEBUG_SERIAL.print(F(" read_bal=")); DEBUG_SERIAL.print(READ_DXL_DURING_BALANCE);
  DEBUG_SERIAL.print(F(" tip_count=")); DEBUG_SERIAL.print(tip_bad_count);
  DEBUG_SERIAL.print(F(" rearm_count=")); DEBUG_SERIAL.print(rearm_good_count);
  DEBUG_SERIAL.println();
}

void handleSerialCommands() {
  while (DEBUG_SERIAL.available()) {
    char c = DEBUG_SERIAL.read();

  if (c == 'p') {
    run_mode = MODE_STATE_PRINT;
    controller_armed = false;
    adaptive_enabled = false;
    controller_safety_paused = false;
    stopMotors();
    DEBUG_SERIAL.println(F("Mode: STATE_PRINT"));
  } else if (c == 't') {
    controller_armed = false;
    adaptive_enabled = false;
    resetControllerStates();
    controller_safety_paused = false;
    torqueOnMotors();
    run_mode = MODE_DXL_STEP_TEST;
    DEBUG_SERIAL.println(F("Mode: DXL_STEP_TEST - wheels must be off ground"));
  } else if (c == 'c') {
    resetControllerStates();
    controller_safety_paused = false;
    adaptive_enabled = false;
    controller_armed = true;
    torqueOnMotors();
    run_mode = MODE_FIXED_CONTROLLER;
    DEBUG_SERIAL.println(F("Mode: FIXED_CONTROLLER"));
  } else if (c == 'a') {
    resetControllerStates();

    // Critical: adaptive mode starts from the exact working fixed gains.
    syncAdaptiveBaselineToFixed();
    resetInverseAdaptiveStatesOnly();

    controller_safety_paused = false;
    adaptive_enabled = true;
    controller_armed = true;
    torqueOnMotors();
    run_mode = MODE_ADAPTIVE_CONTROLLER;
    DEBUG_SERIAL.println(F("Mode: ADAPTIVE_CONTROLLER - inverse adaptive baseline synced to fixed gains"));
  } else if (c == 'x') {
    adaptive_enabled = false;
    resetInverseAdaptiveStatesOnly();
    DEBUG_SERIAL.println(F("Adaptive law disabled and inverse adaptive gains reset."));
  } else if (c == 'r') {
    resetControllerStates();
    DEBUG_SERIAL.println(F("Controller states reset."));
  } else if (c == '0' || c == 's') {
    fullStopToPrintMode();
    DEBUG_SERIAL.println(F("STOP -> STATE_PRINT"));
  } else if (c == 'z') {
    int32_t vel_dummy;
    if (readDxlState(ID_LEFT, pos_left_zero, vel_dummy) &&
        readDxlState(ID_RIGHT, pos_right_zero, vel_dummy)) {
      phi_ref = 0.0f;
      theta_ref = 0.0f;
      resetControllerStates();
      DEBUG_SERIAL.println(F("Re-zeroed wheel positions and controller states."));
    } else {
      DEBUG_SERIAL.println(F("Zero failed: could not read motor positions."));
    }
  } else if (c == 'd') {
    raw_imu_print_enabled = !raw_imu_print_enabled;
    DEBUG_SERIAL.print(F("Raw/verbose printing "));
    DEBUG_SERIAL.println(raw_imu_print_enabled ? F("ON") : F("OFF"));
  } else if (c == 'l') {
    csv_log_enabled = !csv_log_enabled;
    csv_header_printed = false;

    DEBUG_SERIAL.print(F("CSV logging "));
    DEBUG_SERIAL.println(csv_log_enabled ? F("ON") : F("OFF"));
  }
}
}

// ================================================================
// Arduino setup / loop
// ================================================================

void setup() {
  pinMode(13, OUTPUT);
  pinMode(DXL_DIR_PIN, OUTPUT);
  dxlSetRX();

  DEBUG_SERIAL.begin(DEBUG_BAUD);
  delay(1000);

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("Self-Balancing Bot - Inverse Adaptive - Raw IMU + Complementary theta"));
  DEBUG_SERIAL.println(F("Commands: p=print, t=step test, c=fixed, a=adaptive, l=CSV log, d=verbose IMU, x=adaptive off, r=reset, s/0=stop, z=zero"));
  DEBUG_SERIAL.println(F("DXL protection: no DXL reads during balance, WRITE status replies disabled, compact controller logging."));

  initMPU6050();
  DEBUG_SERIAL.println(F("MPU6050 ready."));

  initDynamixels();
  DEBUG_SERIAL.println(F("Dynamixels initialized."));

  stopMotors();
  last_control_us = micros();
}

void loop() {

  handleSerialCommands();

  uint32_t now_us = micros();
  if ((uint32_t)(now_us - last_control_us) < CONTROL_PERIOD_US) {
    return;
  }

  float dt = (now_us - last_control_us) * 1.0e-6f;
  last_control_us = now_us;
  if (dt <= 0.0f || dt > 0.1f) dt = TS_DESIRED;

  updateIMU(dt);
  // imuLogUpdate();   // Added this line to save IMU states for plotting ML

  bool wheel_state_ok = updateWheelStates(false);

  bool balance_mode = (run_mode == MODE_FIXED_CONTROLLER || run_mode == MODE_ADAPTIVE_CONTROLLER);
  if (balance_mode && !READ_DXL_DURING_BALANCE) {
    // Avoid Dynamixel reads during balancing, but still provide the inverse model
    // with an approximate chassis velocity from the last commanded wheel speed.
    float omega_for_xdot = (controller_armed && !controller_safety_paused) ? last_sent_omega_cmd : 0.0f;
    float x_dot_cmd_meas = XDOT_SIGN * r1 * omega_for_xdot;
    x_dot_est = (1.0f - XDOT_CMD_ALPHA) * x_dot_est + XDOT_CMD_ALPHA * x_dot_cmd_meas;
    x_dot = x_dot_est;
  } else if (wheel_state_ok) {
    float x_dot_meas = phi_dot_rad_s * r1;
    x_dot = (1.0f - XDOT_LPF_ALPHA) * x_dot + XDOT_LPF_ALPHA * x_dot_meas;
    x_dot_est = x_dot;
  } else {
    x_dot_est = (1.0f - XDOT_CMD_ALPHA) * x_dot_est;
    x_dot = x_dot_est;
  }

  float omega_cmd = 0.0f;
  bool controller_allowed = updateActiveSafety();

  if (!controller_allowed && isControllerMode()) {
    omega_cmd = 0.0f;
    // Motors are torque-off while safety is paused. Keep mode selected so it
    // can auto-rearm when the bot is moved back upright.
  } else {
    switch (run_mode) {
      case MODE_STATE_PRINT:
        omega_cmd = 0.0f;
        //commandWheelVelocityRadps(0.0f);
        break;

      case MODE_DXL_STEP_TEST:
        omega_cmd = computeDxlStepTestCommand();
        sendWheelCommandRateLimited(omega_cmd, false);
        break;

      case MODE_FIXED_CONTROLLER:
        
        if (controller_armed) {
          omega_cmd = computeFixedController(dt);
          sendWheelCommandRateLimited(omega_cmd, false);
        } else {
          sendWheelCommandRateLimited(0.0f, false);
        }
        break;

      case MODE_ADAPTIVE_CONTROLLER:
        
        if (controller_armed) {
          omega_cmd = computeAdaptiveController(dt);
          sendWheelCommandRateLimited(omega_cmd, false);
        } else {
          sendWheelCommandRateLimited(0.0f, false);
        }
        break;
    }
  }

  uint32_t print_period = csv_log_enabled ? CSV_LOG_PERIOD_MS : PRINT_PERIOD_MS;

  if (millis() - last_print_ms >= print_period) {
    last_print_ms = millis();
    printState(omega_cmd);
  }
}
