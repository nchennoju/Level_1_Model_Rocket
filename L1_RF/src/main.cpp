#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <Adafruit_BMP085.h>
#include "RF24.h"

// ============================================================
// RADIO
// ============================================================
RF24 radio(1, 2);                 // CE, CSN
const byte address[6] = "00001";  // TX pipe

union float_byte {
  float value;
  byte b[4];
};

union short_byte {
  int16_t value;
  byte b[2];
};

// ============================================================
// TELEMETRY FIELDS
// ============================================================
/*
mode encoding:
1   nominal
/3  GPS bad
/5  BMP bad
/7  IMU A bad
/11 IMU B bad
*/
union short_byte mode_u;
union float_byte currentLat;
union float_byte currentLon;
union float_byte currentAlt;
union short_byte currentAccelMag;
union short_byte currentGX;
union short_byte currentGY;
union short_byte currentGZ;
union short_byte currentN;
union short_byte currentE;
union short_byte currentYawNav;
union short_byte checksum_u;

// ============================================================
// SERIAL HEADER
// ============================================================
#define HEADER "The following flight computer was designed and made by Nitish Chennoju\n\nTHIS IMU CONSISTS OF THE FOLLOWING SENSORS\n - MPU6050\n - BMP180\n - UBLOX GPS"
#define HEADER_END "----------- SERIAL START -----------\n\n"

// ============================================================
// IMU / LOOP CONFIG
// ============================================================
#define GYRO_CALIBRATION_PERIOD 2000
#define NUM_IMU 2

#define Frequency 125
#define Sensitivity 65.5
#define Loop_time (1000000 / Frequency)

// IMU I2C addresses
byte MPU6050_I2C_ADDR[NUM_IMU] = {0x68, 0x69};

// Faster telemetry update
const int INTERVAL = 50;  // ms

// GPS gating thresholds
const float GPS_YAW_ANCHOR_SPEED_MPS = 3.0f;
const uint8_t GPS_MIN_SATS = 5;
const float GPS_MAX_HDOP = 3.0f;

// IMU disagreement thresholds
const float G_DISAGREE = 25.0f;  // deg/s sum disagreement threshold
const float A_DISAGREE = 0.9f;   // g sum disagreement threshold

// Faster accel response
const float ACC_LPF_ALPHA = 0.35f;

// Relative altitude baseline
const int BARO_BASELINE_SAMPLES = 50;

// Requested scale compensation
const float ROLL_SCALE  = 3.0f;
const float PITCH_SCALE = 3.0f;
const float YAW_SCALE   = 2.0f;

// ============================================================
// DECLARATIONS
// ============================================================
int  getIndex(byte addr);
bool read_mpu_6050_data(byte addr);
void calibrate_gyro(byte addr);
void config_gyro(byte addr);

static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}
static float computeAltRelFromPressure(float pPa, float p0Pa);
static void  initPressureBaseline();

// ============================================================
// GLOBALS - SENSOR STATE
// ============================================================
long Loop_start;

double temperature[NUM_IMU];
float  Gyro_x[NUM_IMU], Gyro_y[NUM_IMU], Gyro_z[NUM_IMU];
long   Gyro_x_cal[NUM_IMU], Gyro_y_cal[NUM_IMU], Gyro_z_cal[NUM_IMU];
long   Acc_x_cal[NUM_IMU],  Acc_y_cal[NUM_IMU],  Acc_z_cal[NUM_IMU];
float  Accel_x[NUM_IMU], Accel_y[NUM_IMU], Accel_z[NUM_IMU];

float Gyro_pitch = 0, Gyro_roll = 0, Gyro_yaw = 0;
float Gyro_pitch_output = 0, Gyro_roll_output = 0;

unsigned long prvTime = 0, prvCal = 0;
float deltaTime = 0;
float postCal_pitch = 0, postCal_roll = 0, postCal_yaw = 0;
bool runOnce = false;

TinyGPSPlus gps;
Adafruit_BMP085 bmp;

// Baro
float temp = 0, real_alt = 0;
float real_pres = 0;
float p0Pa = 0.0f;
bool p0Init = false;

// Timing
unsigned long start = 0;

// Sensor health
bool GPS_STATUS = true;
bool IMU_A_STATUS = true;
bool IMU_B_STATUS = true;
bool BMP_STATUS = true;

// ============================================================
// KALMAN FILTER
// ============================================================
class KalmanAngle {
public:
  // Tuned a bit hotter for faster response
  float Q_angle = 0.01f;
  float Q_bias  = 0.01f;
  float R_meas  = 0.03f;

  float angle = 0.0f;
  float bias  = 0.0f;
  float P00 = 1, P01 = 0, P10 = 0, P11 = 1;

  float update(float newAngle, float newRate, float dt) {
    float rate = newRate - bias;
    angle += dt * rate;

    P00 += dt * (dt * P11 - P01 - P10 + Q_angle);
    P01 -= dt * P11;
    P10 -= dt * P11;
    P11 += Q_bias * dt;

    float y = newAngle - angle;
    float S = P00 + R_meas;

    float K0 = P00 / S;
    float K1 = P10 / S;

    angle += K0 * y;
    bias  += K1 * y;

    float P00_temp = P00;
    float P01_temp = P01;

    P00 -= K0 * P00_temp;
    P01 -= K0 * P01_temp;
    P10 -= K1 * P00_temp;
    P11 -= K1 * P01_temp;

    return angle;
  }

  void setAngle(float a) { angle = a; }
};

// ============================================================
// ATTITUDE FUSION STATE
// ============================================================
KalmanAngle kfRoll0, kfPitch0;
KalmanAngle kfRoll1, kfPitch1;
bool attInit0 = false, attInit1 = false;

float ax0_f = 0, ay0_f = 0, az0_f = 1;
float ax1_f = 0, ay1_f = 0, az1_f = 1;

float last_gx = 0, last_gy = 0, last_gz = 0;

// ============================================================
// NAV STATE
// ============================================================
double refLat = 0.0, refLon = 0.0;
bool refInit = false;

float yaw_deg = 0.0f;
bool yawInit = false;

float rawN = 0.0f;
float rawE = 0.0f;
double rawLat = 0.0;
double rawLon = 0.0;
float rawAltRel = 0.0f;

float fused_alt = 0.0f;
double fused_lat = 0.0;
double fused_lon = 0.0;

// ============================================================
// SETUP
// ============================================================
void setup() {
  mode_u.value = 1;
  currentLat.value = 0;
  currentLon.value = 0;
  currentAlt.value = 0;
  currentAccelMag.value = 0;
  currentGX.value = 0;
  currentGY.value = 0;
  currentGZ.value = 0;
  currentN.value = 0;
  currentE.value = 0;
  currentYawNav.value = 0;
  checksum_u.value = 0;

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  delay(5000);
  Wire.begin();

  Serial.begin(115200);
  Serial.println(HEADER);
  Serial.println(HEADER_END);

  Serial1.begin(9600);

  unsigned long gpsStart = millis();
  while (millis() - gpsStart < 10000UL) {
    while (Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
    if (gps.location.isValid() &&
        gps.satellites.isValid() &&
        gps.satellites.value() >= GPS_MIN_SATS) {
      break;
    }
  }

  if (gps.location.isValid()) {
    refLat = gps.location.lat();
    refLon = gps.location.lng();
    refInit = true;

    rawLat = refLat;
    rawLon = refLon;
    fused_lat = refLat;
    fused_lon = refLon;
  }

  if (!bmp.begin(BMP085_ULTRAHIGHRES)) {
    Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
    BMP_STATUS = false;
  } else {
    initPressureBaseline();
  }

  start = millis();
  delay(500);

  for (int i = 0; i < NUM_IMU; i++) {
    config_gyro(MPU6050_I2C_ADDR[i]);
  }

  for (int i = 0; i < NUM_IMU; i++) {
    calibrate_gyro(MPU6050_I2C_ADDR[i]);
  }

  prvTime = micros();
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  GPS_STATUS = false;
  if (gps.location.isValid()) {
    uint8_t sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
    float hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 99.0f;
    GPS_STATUS = (sats >= GPS_MIN_SATS) && (hdop <= GPS_MAX_HDOP);
  }

  if (!runOnce) {
    prvCal = micros();

    for (int j = 0; j < 2000; j++) {
      float gx_sum = 0, gy_sum = 0, gz_sum = 0;

      for (int i = 0; i < NUM_IMU; i++) {
        bool ok = read_mpu_6050_data(MPU6050_I2C_ADDR[i]);
        if (!ok) {
          if (i == 0) IMU_A_STATUS = false;
          if (i == 1) IMU_B_STATUS = false;
          continue;
        }

        Gyro_x[i] -= Gyro_x_cal[i];
        Gyro_y[i] -= Gyro_y_cal[i];
        Gyro_z[i] -= Gyro_z_cal[i];

        gx_sum += Gyro_x[i];
        gy_sum += Gyro_y[i];
        gz_sum += Gyro_z[i];
      }

      float gx_avg = gx_sum * 0.5f;
      float gy_avg = gy_sum * 0.5f;
      float gz_avg = gz_sum * 0.5f;

      deltaTime = (micros() - prvTime) / 1000000.0f;
      prvTime = micros();
      if (deltaTime <= 0) deltaTime = 0.001f;

      Gyro_pitch += gy_avg * deltaTime;
      Gyro_roll  += gx_avg * deltaTime;
      Gyro_yaw   += -gz_avg * deltaTime;
    }

    float calDur = (micros() - prvCal) / 1000000.0f;
    if (calDur <= 0) calDur = 1.0f;

    postCal_pitch = Gyro_pitch / calDur;
    postCal_roll  = Gyro_roll  / calDur;
    postCal_yaw   = Gyro_yaw   / calDur;

    Gyro_pitch = 0;
    Gyro_roll  = 0;
    Gyro_yaw   = 0;

    runOnce = true;
  }

  bool imu0_ok = read_mpu_6050_data(MPU6050_I2C_ADDR[0]);
  bool imu1_ok = read_mpu_6050_data(MPU6050_I2C_ADDR[1]);

  IMU_A_STATUS = imu0_ok;
  IMU_B_STATUS = imu1_ok;

  Gyro_x[0] -= Gyro_x_cal[0]; Gyro_y[0] -= Gyro_y_cal[0]; Gyro_z[0] -= Gyro_z_cal[0];
  Gyro_x[1] -= Gyro_x_cal[1]; Gyro_y[1] -= Gyro_y_cal[1]; Gyro_z[1] -= Gyro_z_cal[1];

  float gx0 = Gyro_x[0], gy0 = Gyro_y[0], gz0 = Gyro_z[0];
  float gx1 = Gyro_x[1], gy1 = Gyro_y[1], gz1 = Gyro_z[1];

  float ax0 = Accel_x[0], ay0 = Accel_y[0], az0 = Accel_z[0];
  float ax1 = Accel_x[1], ay1 = Accel_y[1], az1 = Accel_z[1];

  deltaTime = (micros() - prvTime) / 1000000.0f;
  prvTime = micros();
  if (deltaTime <= 0) deltaTime = 0.001f;
  deltaTime = clampf(deltaTime, 0.001f, 0.05f);

  float dG = fabs(gx0 - gx1) + fabs(gy0 - gy1) + fabs(gz0 - gz1);
  float dA = fabs(ax0 - ax1) + fabs(ay0 - ay1) + fabs(az0 - az1);

  float gx_use, gy_use, gz_use;
  if (dG > G_DISAGREE) {
    float e0 = fabs(gx0 - last_gx) + fabs(gy0 - last_gy) + fabs(gz0 - last_gz);
    float e1 = fabs(gx1 - last_gx) + fabs(gy1 - last_gy) + fabs(gz1 - last_gz);
    if (e0 <= e1) { gx_use = gx0; gy_use = gy0; gz_use = gz0; }
    else          { gx_use = gx1; gy_use = gy1; gz_use = gz1; }
  } else {
    gx_use = 0.5f * (gx0 + gx1);
    gy_use = 0.5f * (gy0 + gy1);
    gz_use = 0.5f * (gz0 + gz1);
  }
  last_gx = gx_use;
  last_gy = gy_use;
  last_gz = gz_use;

  Gyro_yaw += -gz_use * deltaTime;
  Gyro_yaw -= postCal_yaw * deltaTime;

  if (!yawInit) {
    yaw_deg = Gyro_yaw;
    yawInit = true;
  }
  yaw_deg += (-gz_use - postCal_yaw) * deltaTime;

  ax0_f += ACC_LPF_ALPHA * (ax0 - ax0_f);
  ay0_f += ACC_LPF_ALPHA * (ay0 - ay0_f);
  az0_f += ACC_LPF_ALPHA * (az0 - az0_f);

  ax1_f += ACC_LPF_ALPHA * (ax1 - ax1_f);
  ay1_f += ACC_LPF_ALPHA * (ay1 - ay1_f);
  az1_f += ACC_LPF_ALPHA * (az1 - az1_f);

  float accMag0 = sqrt(ax0_f * ax0_f + ay0_f * ay0_f + az0_f * az0_f);
  float accMag1 = sqrt(ax1_f * ax1_f + ay1_f * ay1_f + az1_f * az1_f);
  if (accMag0 < 1e-6f) accMag0 = 1e-6f;
  if (accMag1 < 1e-6f) accMag1 = 1e-6f;

  float Accel_pitch0 = asin(ax0_f / accMag0) * RAD_TO_DEG - (-0.2f);
  float Accel_roll0  = asin(ay0_f / accMag0) * RAD_TO_DEG - ( 1.1f);

  float Accel_pitch1 = asin(ax1_f / accMag1) * RAD_TO_DEG - (-0.2f);
  float Accel_roll1  = asin(ay1_f / accMag1) * RAD_TO_DEG - ( 1.1f);

  if (!attInit0) {
    kfPitch0.setAngle(Accel_pitch0);
    kfRoll0.setAngle(Accel_roll0);
    attInit0 = true;
  }
  if (!attInit1) {
    kfPitch1.setAngle(Accel_pitch1);
    kfRoll1.setAngle(Accel_roll1);
    attInit1 = true;
  }

  float err0 = fabs(accMag0 - 1.0f);
  float err1 = fabs(accMag1 - 1.0f);

  float Rbase = 0.02f;
  float Rmax  = 5.0f;

  float R0 = clampf(Rbase + 35.0f * err0 * err0, Rbase, Rmax);
  float R1 = clampf(Rbase + 35.0f * err1 * err1, Rbase, Rmax);

  if (dA > A_DISAGREE) {
    if (err0 > err1) R0 = Rmax;
    else             R1 = Rmax;
  }

  kfPitch0.R_meas = R0; kfRoll0.R_meas = R0;
  kfPitch1.R_meas = R1; kfRoll1.R_meas = R1;

  float kalPitch0 = kfPitch0.update(Accel_pitch0, (gy0 - postCal_pitch), deltaTime);
  float kalRoll0  = kfRoll0.update(Accel_roll0,  (gx0 - postCal_roll),  deltaTime);

  float kalPitch1 = kfPitch1.update(Accel_pitch1, (gy1 - postCal_pitch), deltaTime);
  float kalRoll1  = kfRoll1.update(Accel_roll1,  (gx1 - postCal_roll),  deltaTime);

  float w0 = 1.0f / (0.05f + err0);
  float w1 = 1.0f / (0.05f + err1);

  if (fabs(kalPitch0 - kalPitch1) > 12.0f) {
    if (err0 > err1) w0 *= 0.1f;
    else             w1 *= 0.1f;
  }
  if (fabs(kalRoll0 - kalRoll1) > 12.0f) {
    if (err0 > err1) w0 *= 0.1f;
    else             w1 *= 0.1f;
  }

  float inv = 1.0f / (w0 + w1);
  Gyro_pitch_output = (w0 * kalPitch0 + w1 * kalPitch1) * inv;
  Gyro_roll_output  = (w0 * kalRoll0  + w1 * kalRoll1 ) * inv;

  if (gps.location.isUpdated() && gps.location.isValid() && GPS_STATUS) {
    if (!refInit) {
      refLat = gps.location.lat();
      refLon = gps.location.lng();
      refInit = true;
    }

    rawLat = gps.location.lat();
    rawLon = gps.location.lng();

    const double Rearth = 6378137.0;
    double dLat = (rawLat - refLat) * (3.14159265358979323846 / 180.0);
    double dLon = (rawLon - refLon) * (3.14159265358979323846 / 180.0);

    rawN = (float)(dLat * Rearth);
    rawE = (float)(dLon * Rearth * cos(refLat * (3.14159265358979323846 / 180.0)));

    if (gps.speed.isValid() && gps.course.isValid()) {
      float spd = gps.speed.mps();
      if (spd > GPS_YAW_ANCHOR_SPEED_MPS) {
        yaw_deg = gps.course.deg();
      }
    }
  }

  fused_lat = rawLat;
  fused_lon = rawLon;

  if ((millis() - start) > (unsigned long)INTERVAL) {
    start = millis();

    if (BMP_STATUS) {
      temp = bmp.readTemperature();
      real_pres = (float)bmp.readPressure();
      real_alt = p0Init ? computeAltRelFromPressure(real_pres, p0Pa) : 0.0f;
      rawAltRel = real_alt;
      fused_alt = rawAltRel;
    } else {
      rawAltRel = 0.0f;
      fused_alt = 0.0f;
    }

    float ax_print = 0.5f * (ax0 + ax1);
    float ay_print = 0.5f * (ay0 + ay1);
    float az_print = 0.5f * (az0 + az1);
    float accelMag = sqrt(ax_print * ax_print + ay_print * ay_print + az_print * az_print);

    // Requested scaling patch
    float roll_tx  = Gyro_roll_output * ROLL_SCALE;
    float pitch_tx = Gyro_pitch_output * PITCH_SCALE;
    float yaw_tx   = Gyro_yaw * YAW_SCALE;

    int32_t modeVal = 1;
    if (!GPS_STATUS)   modeVal *= 3;
    if (!BMP_STATUS)   modeVal *= 5;
    if (!IMU_A_STATUS) modeVal *= 7;
    if (!IMU_B_STATUS) modeVal *= 11;
    mode_u.value = (int16_t)modeVal;

    currentLat.value = (float)rawLat;
    currentLon.value = (float)rawLon;
    currentAlt.value = (float)rawAltRel;
    currentAccelMag.value = (int16_t)(accelMag * 10.0f);

    currentGX.value = (int16_t)roll_tx;
    currentGY.value = (int16_t)pitch_tx;
    currentGZ.value = (int16_t)yaw_tx;

    currentN.value = (int16_t)clampf(rawN, -32768.0f, 32767.0f);
    currentE.value = (int16_t)clampf(rawE, -32768.0f, 32767.0f);
    currentYawNav.value = (int16_t)yaw_deg;

    String tmp = "{";
    tmp += "\"ts\": " + String((millis() / 1000.0), 3) + ",";
    tmp += "\"alt_rel_m\": " + String(rawAltRel, 2) + ",";
    tmp += "\"roll\": " + String(roll_tx, 2) + ",";
    tmp += "\"pitch\": " + String(pitch_tx, 2) + ",";
    tmp += "\"yaw\": " + String(yaw_tx, 2) + ",";
    tmp += "\"yawNav\": " + String(yaw_deg, 2) + ",";
    tmp += "\"lat\": " + String(rawLat, 6) + ",";
    tmp += "\"lon\": " + String(rawLon, 6) + ",";
    tmp += "\"N\": " + String(rawN, 2) + ",";
    tmp += "\"E\": " + String(rawE, 2) + ",";
    tmp += "\"accel_g\": " + String(accelMag, 2) + ",";
    tmp += "\"mode\": " + String(mode_u.value) + "}";
    Serial.println(tmp);

    byte text[30];
    int idx = 0;

    for (int i = 0; i < 2; i++) text[idx++] = mode_u.b[i];
    for (int i = 0; i < 4; i++) text[idx++] = currentLat.b[i];
    for (int i = 0; i < 4; i++) text[idx++] = currentLon.b[i];
    for (int i = 0; i < 4; i++) text[idx++] = currentAlt.b[i];
    for (int i = 0; i < 2; i++) text[idx++] = currentAccelMag.b[i];
    for (int i = 0; i < 2; i++) text[idx++] = currentGX.b[i];
    for (int i = 0; i < 2; i++) text[idx++] = currentGY.b[i];
    for (int i = 0; i < 2; i++) text[idx++] = currentGZ.b[i];
    for (int i = 0; i < 2; i++) text[idx++] = currentN.b[i];
    for (int i = 0; i < 2; i++) text[idx++] = currentE.b[i];
    for (int i = 0; i < 2; i++) text[idx++] = currentYawNav.b[i];

    checksum_u.value = 0;
    for (int i = 0; i < 28; i++) {
      checksum_u.value += text[i];
    }

    for (int i = 0; i < 2; i++) text[idx++] = checksum_u.b[i];

    radio.write(&text, sizeof(text));
  }
}

// ============================================================
// HELPERS
// ============================================================
int getIndex(byte addr) {
  for (int i = 0; i < NUM_IMU; i++) {
    if (MPU6050_I2C_ADDR[i] == addr) return i;
  }
  return -1;
}

void config_gyro(byte addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(addr);
  Wire.write(0x1C);
  Wire.write(0x18);   // +/-16g
  Wire.endTransmission();

  Wire.beginTransmission(addr);
  Wire.write(0x1B);
  Wire.write(0x18);   // 2000 dps
  Wire.endTransmission();
}

void calibrate_gyro(byte addr) {
  int idx = getIndex(addr);
  if (idx < 0) return;

  Serial.print("Calibrating Gyro: ");
  Serial.print(addr);
  Serial.println("...");

  Gyro_x_cal[idx] = 0;
  Gyro_y_cal[idx] = 0;
  Gyro_z_cal[idx] = 0;
  Acc_x_cal[idx]  = 0;
  Acc_y_cal[idx]  = 0;
  Acc_z_cal[idx]  = 0;

  for (int counter = 0; counter < GYRO_CALIBRATION_PERIOD; counter++) {
    Loop_start = micros();

    if (read_mpu_6050_data(addr)) {
      Gyro_x_cal[idx] += Gyro_x[idx];
      Gyro_y_cal[idx] += Gyro_y[idx];
      Gyro_z_cal[idx] += Gyro_z[idx];

      Acc_x_cal[idx]  += Accel_x[idx];
      Acc_y_cal[idx]  += Accel_y[idx];
      Acc_z_cal[idx]  += Accel_z[idx];
    }

    while (micros() - Loop_start < Loop_time);
  }

  Gyro_x_cal[idx] /= GYRO_CALIBRATION_PERIOD;
  Gyro_y_cal[idx] /= GYRO_CALIBRATION_PERIOD;
  Gyro_z_cal[idx] /= GYRO_CALIBRATION_PERIOD;

  Acc_x_cal[idx]  /= GYRO_CALIBRATION_PERIOD;
  Acc_y_cal[idx]  /= GYRO_CALIBRATION_PERIOD;
  Acc_z_cal[idx]  /= GYRO_CALIBRATION_PERIOD;

  Serial.println("Done");
}

bool read_mpu_6050_data(byte addr) {
  int idx = getIndex(addr);
  if (idx < 0) return false;

  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  Wire.requestFrom((int)addr, 14, true);

  unsigned long t0 = millis();
  while (Wire.available() < 14) {
    if (millis() - t0 > 20) {
      return false;
    }
  }

  Accel_x[idx] = (int16_t)(Wire.read() << 8 | Wire.read()) / 2048.0f;
  Accel_y[idx] = (int16_t)(Wire.read() << 8 | Wire.read()) / 2048.0f;
  Accel_z[idx] = (int16_t)(Wire.read() << 8 | Wire.read()) / 2048.0f;

  temperature[idx] = ((int16_t)(Wire.read() << 8 | Wire.read()) / 340.0f) + 36.53f;

  Gyro_x[idx] = (int16_t)(Wire.read() << 8 | Wire.read()) / 16.4f;
  Gyro_y[idx] = (int16_t)(Wire.read() << 8 | Wire.read()) / 16.4f;
  Gyro_z[idx] = (int16_t)(Wire.read() << 8 | Wire.read()) / 16.4f;

  return true;
}

static float computeAltRelFromPressure(float pPa, float p0Pa_local) {
  if (pPa <= 0 || p0Pa_local <= 0) return 0.0f;
  return 44330.0f * (1.0f - powf(pPa / p0Pa_local, 0.19029495f));
}

static void initPressureBaseline() {
  float sum = 0.0f;
  for (int i = 0; i < BARO_BASELINE_SAMPLES; i++) {
    sum += (float)bmp.readPressure();
    delay(INTERVAL);
  }
  p0Pa = sum / (float)BARO_BASELINE_SAMPLES;
  p0Init = true;

  Serial.print("BMP180 baseline P0 (Pa): ");
  Serial.println(p0Pa, 0);
}