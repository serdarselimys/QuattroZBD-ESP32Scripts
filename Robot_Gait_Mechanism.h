#ifndef ROBOT_GAIT_MECHANISM_H
#define ROBOT_GAIT_MECHANISM_H

const bool DEBUG_LEVEL_PRINT = false;
const bool DEBUG_MENU_INPUT  = false;
const bool DEBUG_IMU_PRINT   = false;

// ============================================================
// GEOMETRY LAYOUT ARRAYS -- 
// ============================================================
const char* LEG_ORDER[4] = {"FL", "RL", "FR", "RR"};
const float LEG_ROOTS[4][2] = {
    {0.15, 0.10}, {-0.15, 0.10},
    {0.15, -0.10}, {-0.15, -0.10}
};

const float LEVEL_CENTER_X = (LEG_ROOTS[0][0] + LEG_ROOTS[1][0] + LEG_ROOTS[2][0] + LEG_ROOTS[3][0]) / 4.0f;
const float LEVEL_CENTER_Y = (LEG_ROOTS[0][1] + LEG_ROOTS[1][1] + LEG_ROOTS[2][1] + LEG_ROOTS[3][1]) / 4.0f;

// ============================================================
// GAIT PROFILE DATA -- embedded from merged_params.csv
// ============================================================
struct QuadGaitRow {
    char  gait[10];        // "walk" | "trot" | "sideway" | "diagonal" | "spin"
    float target_vel;
    float step_amplitude;
    float frequency;
    float sway_amp;        // 0 where the CSV left it blank
    float duty;            // only meaningful for walk/trot/diagonal rows
};

#define QUAD_GAIT_MATRIX_SIZE 16
const QuadGaitRow quadGaitMatrix[QUAD_GAIT_MATRIX_SIZE] = {
    { "walk",     0.150, 0.0371120074943428, 1.55710151984365, 0.006591683384597,   0.6  },
    { "walk",     0.125, 0.0394551727012075, 1.36598654722308, 0.00642934695318449, 0.6  },
    { "walk",     0.100, 0.0300322148398475, 1.38687132732155, 0.00835485319848285, 0.6  },
    { "walk",     0.075, 0.0229425195579065, 1.27086018915593, 0.00728120205420466, 0.6  },
    { "trot",     0.150, 0.0481821166896783, 1.25219516374191, 0.00225095410552645, 0.5  },
    { "trot",     0.125, 0.0436526114813749, 1.10920676344033, 0.00437375799988358, 0.5  },
    { "trot",     0.100, 0.0281236024571577, 1.36044152842148, 0.00346080006108283, 0.5  },
    { "trot",     0.075, 0.0223166496395632, 1.20608157179505, 0.00449572398645411, 0.5  },
    { "spin",     0.700, 0.0798614106075095, 1.57665264500454, 0.0,                 0.5  },
    { "spin",     0.500, 0.0678355614088876, 1.38437625167692, 0.0,                 0.5  },
    { "spin",     0.300, 0.0602050560748612, 1.02543638143057, 0.0,                 0.5  },
    { "sideway",  0.100, 0.0160010240579149, 1.15401399766421, 0.0,                 0.5  },
    { "sideway",  0.075, 0.0171741894261218, 0.802613137716824,0.0,                 0.5  },
    { "diagonal", 0.125, 0.0261229162505303, 1.05367510286441, 0.0,                 0.55 },
    { "diagonal", 0.100, 0.0220314839754043, 1.03857760874668, 0.0,                 0.55 },
    { "diagonal", 0.075, 0.017229623078828,  1.01839877970741, 0.0,                 0.55 },
};

// Two leg-timing patterns (phase offset per leg, LEG_ORDER-indexed).
const float TROT_OFFSETS[4] = { 0.0f, 0.5f, 0.5f, 0.0f };
const float WALK_OFFSETS[4] = { 0.75f, 0.0f, 0.25f, 0.5f };
const float TROT_DUTY = 0.5f;

const float DIAGONAL_DEFAULT_DUTY = 0.55f;
const float SIDEWAY_DEFAULT_DUTY  = 0.5f;

const float PARAM_SMOOTH_TAU = 0.20f;
const float GAIT_BLEND_TAU   = 0.25f;
const float WALK_ENTER_SHARE = 0.60f;  // crossfade share used on a live trot<->walk switch
const float WALK_EXIT_SHARE  = 0.40f;

// ============================================================
// SERVO I/O + FLASH PERSISTENCE
// ============================================================
void saveOffsetsToFlash() {
    preferences.begin("quad-cal", false);
    for (int i = 0; i < 12; i++) {
        String key = "off_" + String(i);
        preferences.putFloat(key.c_str(), servoOffsets[i]);
    }
    preferences.end();
    Serial.println("Calibration metrics permanently stored to Flash NVS.");
}

void loadOffsetsFromFlash() {
    preferences.begin("quad-cal", true);
    for (int i = 0; i < 12; i++) {
        String key = "off_" + String(i);
        servoOffsets[i] = preferences.getFloat(key.c_str(), servoOffsets[i]);
    }
    preferences.end();
    Serial.println("Saved system offsets successfully deployed from Flash.");
}

void shutDownServosHardware() {
    for (int pin = 0; pin < 16; pin++) {
        pwm1.setPWM(pin, 0, 4096);
    }
    servosArePowered = false;
    Serial.println("Servos safe: PWM outputs completely disabled.");
}

// 12 servos, ONE PCA9685 board (0x40), matching the physical wiring:
//   FR1,FR2,FR3 -> board pins 1,2,3   (channels 0,1,2)
//   FL1,FL2,FL3 -> board pins 5,6,7   (channels 4,5,6)
//   RR1,RR2,RR3 -> board pins 9,10,11 (channels 8,9,10)
//   RL1,RL2,RL3 -> board pins 13,14,15 (channels 12,13,14)

void setServo(int id, float angle) {
    if (id < 0 || id >= 12) return;
    int group        = id / 3;              // 0=FR, 1=FL, 2=RR, 3=RL
    int withinLeg     = id % 3;              // 0=knee, 1=thigh, 2=hip
    int physicalPin   = group * 4 + withinLeg;   // 4-channel stride leaves 1 gap per leg
    float calibratedAngle = constrain(angle + servoOffsets[id], 0.0, 180.0);
    float preciseMicroseconds = USMIN + (calibratedAngle * (2000.0 / 180.0));
    pwm1.writeMicroseconds(physicalPin, (int)preciseMicroseconds);
}


inline void legServoIds(const String &legName, int &kneeHWID, int &thighHWID, int &hipHWID) {
    if      (legName == "FR") { kneeHWID = 0;  thighHWID = 1;  hipHWID = 2;  }
    else if (legName == "FL") { kneeHWID = 3;  thighHWID = 4;  hipHWID = 5;  }
    else if (legName == "RR") { kneeHWID = 6;  thighHWID = 7;  hipHWID = 8;  }
    else if (legName == "RL") { kneeHWID = 9;  thighHWID = 10; hipHWID = 11; }
    else { kneeHWID = thighHWID = hipHWID = -1; }
}

// ---- IMU calibration persistence. ----
bool imuCalCacheLoaded = false;

struct ImuCalSnapshot {
    int   pKind, rKind, pGAxis, rGAxis;
    float pSign, rSign, pGSign, rGSign;
    float pBias, rBias, gBiasP, gBiasR;
} imuCalCache;

void saveImuCalToFlash() {
    preferences.begin("quad-imucal", false);
    preferences.putInt("pKind",  imuAccelPitchKind);
    preferences.putFloat("pSign", imuAccelPitchSign);
    preferences.putInt("rKind",  imuAccelRollKind);
    preferences.putFloat("rSign", imuAccelRollSign);
    preferences.putInt("pGAxis", imuGyroPitchAxis);
    preferences.putFloat("pGSign", imuGyroPitchSign);
    preferences.putInt("rGAxis", imuGyroRollAxis);
    preferences.putFloat("rGSign", imuGyroRollSign);
    preferences.putFloat("pBias", imuPitchBias);
    preferences.putFloat("rBias", imuRollBias);
    preferences.putFloat("gBiasP", imuGyroBiasPitch);
    preferences.putFloat("gBiasR", imuGyroBiasRoll);
    preferences.putBool("valid", true);
    preferences.end();
    Serial.println("[Cal] IMU calibration cached to Flash NVS.");
}

void loadImuCalFromFlash() {
    preferences.begin("quad-imucal", true);
    bool valid = preferences.getBool("valid", false);
    if (valid) {
        imuAccelPitchKind = preferences.getInt("pKind", imuAccelPitchKind);
        imuAccelPitchSign = preferences.getFloat("pSign", imuAccelPitchSign);
        imuAccelRollKind  = preferences.getInt("rKind", imuAccelRollKind);
        imuAccelRollSign  = preferences.getFloat("rSign", imuAccelRollSign);
        imuGyroPitchAxis  = preferences.getInt("pGAxis", imuGyroPitchAxis);
        imuGyroPitchSign  = preferences.getFloat("pGSign", imuGyroPitchSign);
        imuGyroRollAxis   = preferences.getInt("rGAxis", imuGyroRollAxis);
        imuGyroRollSign   = preferences.getFloat("rGSign", imuGyroRollSign);
        imuPitchBias      = preferences.getFloat("pBias", 0.0f);
        imuRollBias       = preferences.getFloat("rBias", 0.0f);
        imuGyroBiasPitch  = preferences.getFloat("gBiasP", 0.0f);
        imuGyroBiasRoll   = preferences.getFloat("gBiasR", 0.0f);
        imuCalCache.pKind  = imuAccelPitchKind; imuCalCache.pSign  = imuAccelPitchSign;
        imuCalCache.rKind  = imuAccelRollKind;  imuCalCache.rSign  = imuAccelRollSign;
        imuCalCache.pGAxis = imuGyroPitchAxis;  imuCalCache.pGSign = imuGyroPitchSign;
        imuCalCache.rGAxis = imuGyroRollAxis;   imuCalCache.rGSign = imuGyroRollSign;
        imuCalCache.pBias  = imuPitchBias;      imuCalCache.rBias  = imuRollBias;
        imuCalCache.gBiasP = imuGyroBiasPitch;  imuCalCache.gBiasR = imuGyroBiasRoll;
        imuCalCacheLoaded = true;
        Serial.println("[Cal] Loaded cached IMU calibration from Flash (fallback only -- fresh dance still runs).");
    } else {
        Serial.println("[Cal] No cached IMU calibration found -- bow/tilt dance will run after standing.");
    }
    preferences.end();
}

// ============================================================
// IMU AXIS REMAP -- ACCEL ANGLE COMPUTATION 
// ============================================================
float computeAccelAngle(int kind, float sign, float ax, float ay, float az) {
    float angle;
    switch (kind) {
        case 0: angle = -atan2(ay, az); break;
        case 1: angle =  atan2(ax, az); break;
        case 2: angle =  atan2(ax, sqrt(ay * ay + az * az)); break;
        case 3: angle =  atan2(ay, sqrt(ax * ax + az * az)); break;
        default: angle = 0.0f; break;
    }
    return sign * angle;
}

float readRawGyroAxis(int axis, float gx, float gy, float gz) {
    switch (axis) {
        case 0: return gx;
        case 1: return gy;
        case 2: return gz;
        default: return 0.0f;
    }
}

// ============================================================
// BALANCE SENSING -- GYRO BIAS CALIBRATION 
// ============================================================
void calibrateGyroBias() {
    sensors_event_t a, g_imu, temp;
    double calibSumX = 0.0, calibSumY = 0.0, calibAccelSum = 0.0;
    int calibCount = 0;
    while (calibCount < GYRO_CALIB_SAMPLES) {
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            mpu.getEvent(&a, &g_imu, &temp);
            calibSumX += g_imu.gyro.x;
            calibSumY += g_imu.gyro.y;
            calibAccelSum += sqrt(a.acceleration.x * a.acceleration.x +
                                  a.acceleration.y * a.acceleration.y +
                                  a.acceleration.z * a.acceleration.z);
            xSemaphoreGive(i2cMutex);
            calibCount++;
        }
        vTaskDelay(pdMS_TO_TICKS((int)GYRO_CALIB_SAMPLE_MS));
    }
    gyroBiasX = (float)(calibSumX / calibCount);
    gyroBiasY = (float)(calibSumY / calibCount);
    accelRestMag = (float)(calibAccelSum / calibCount);
    imuCalibrationDone = true;
    Serial.printf("[IMU] Gyro bias X:%.5f Y:%.5f | Accel rest mag:%.3f (m/s2)\n",
                  gyroBiasX, gyroBiasY, accelRestMag);
}

// ============================================================
// KALMAN FILTER (unchanged -- per-axis, degrees-space)
// ============================================================
struct KalmanState {
    float angle = 0.0f;
    float bias = 0.0f;
    float P[2][2] = {{0, 0}, {0, 0}};
};
KalmanState kalmanPitch, kalmanRoll;

float kalmanUpdateDeg(KalmanState &kf, float newAngleDeg, float rateDegS, float dt) {
    kf.angle += dt * (rateDegS - kf.bias);
    kf.P[0][0] += dt * (dt * kf.P[1][1] - kf.P[0][1] - kf.P[1][0] + KALMAN_Q_ANGLE);
    kf.P[0][1] -= dt * kf.P[1][1];
    kf.P[1][0] -= dt * kf.P[1][1];
    kf.P[1][1] += KALMAN_Q_BIAS * dt;

    float S = kf.P[0][0] + KALMAN_R_MEASURE;
    float K0 = kf.P[0][0] / S;
    float K1 = kf.P[1][0] / S;
    float y = newAngleDeg - kf.angle;
    kf.angle += K0 * y;
    kf.bias  += K1 * y;

    float P00 = kf.P[0][0];
    float P01 = kf.P[0][1];
    kf.P[0][0] -= K0 * P00;
    kf.P[0][1] -= K0 * P01;
    kf.P[1][0] -= K1 * P00;
    kf.P[1][1] -= K1 * P01;

    return kf.angle;
}

void resetKalmanState(KalmanState &kf) {
    kf.angle = 0.0f;
    kf.bias = 0.0f;
    kf.P[0][0] = 0.0f; kf.P[0][1] = 0.0f;
    kf.P[1][0] = 0.0f; kf.P[1][1] = 0.0f;
}

// ============================================================
// BALANCE SENSING -- per-telemetry-tick (unchanged)
// ============================================================
void updateIMUAndBalance() {
    static unsigned long lastImuTime  = millis();
    static unsigned long lastDiagTime = 0;
    sensors_event_t a, g_imu, temp;

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        mpu.getEvent(&a, &g_imu, &temp);
        imu_ax = a.acceleration.x;
        imu_ay = a.acceleration.y;
        imu_az = a.acceleration.z;
        raw_gx = g_imu.gyro.x;
        raw_gy = g_imu.gyro.y;
        raw_gz = g_imu.gyro.z;

        unsigned long now = millis();
        float imu_dt = constrain((now - lastImuTime) / 1000.0f, 0.001f, 0.10f);
        lastImuTime = now;

        float pitch_accel_raw = computeAccelAngle(imuAccelPitchKind, imuAccelPitchSign, imu_ax, imu_ay, imu_az) - imuPitchBias - imuUserPitchZero;
        float roll_accel_raw  = computeAccelAngle(imuAccelRollKind,  imuAccelRollSign,  imu_ax, imu_ay, imu_az) - imuRollBias  - imuUserRollZero;

        float rawPitchGyro = readRawGyroAxis(imuGyroPitchAxis, raw_gx, raw_gy, raw_gz);
        float rawRollGyro  = readRawGyroAxis(imuGyroRollAxis,  raw_gx, raw_gy, raw_gz);
        float gyro_pitch_rate = imuGyroPitchSign * rawPitchGyro - imuGyroBiasPitch;
        float gyro_roll_rate  = imuGyroRollSign  * rawRollGyro  - imuGyroBiasRoll;

        float pitchDeg = kalmanUpdateDeg(kalmanPitch, degrees(pitch_accel_raw), degrees(gyro_pitch_rate), imu_dt);
        float rollDeg  = kalmanUpdateDeg(kalmanRoll,  degrees(roll_accel_raw),  degrees(gyro_roll_rate),  imu_dt);
        body_pitch_filtered = radians(pitchDeg);
        body_roll_filtered  = radians(rollDeg);

        if (DEBUG_IMU_PRINT && millis() - lastDiagTime >= 200) {
            lastDiagTime = millis();
            Serial.printf(
                "[IMU] ax:%6.3f ay:%6.3f az:%6.3f | "
                "roll_accel:%6.2f deg pitch_accel:%6.2f deg | "
                "ROLL_KF:%6.2f deg PITCH_KF:%6.2f deg | "
                "kfBiasR:%+7.4f kfBiasP:%+7.4f deg/s\n",
                imu_ax, imu_ay, imu_az,
                degrees(roll_accel_raw),
                degrees(pitch_accel_raw),
                rollDeg,
                pitchDeg,
                kalmanRoll.bias,
                kalmanPitch.bias
            );
        }

        xSemaphoreGive(i2cMutex);
    }
}

// ============================================================
// FOOTING GATE (unchanged)
// ============================================================
bool footingIsStable() {
    float accelMag = sqrt(imu_ax * imu_ax + imu_ay * imu_ay + imu_az * imu_az);
    float gyroMag  = sqrt(raw_gx * raw_gx + raw_gy * raw_gy + raw_gz * raw_gz);
    bool accelOk = fabs(accelMag - 9.81f) < FOOTING_ACCEL_TOL_MS2;
    bool gyroOk  = gyroMag < FOOTING_GYRO_MAX_RAD_S;
    return accelOk && gyroOk;
}

// ============================================================
// REACH MARGIN (unchanged)
// ============================================================
float currentReachMargin() {
    float nominalExtension = fabs(current_body_height - BODY_OFFSET);
    float margin = LEG_MAX_REACH - nominalExtension;
    return margin > 0.0f ? margin : 0.0f;
}

// ============================================================
// BODY LEVELING (unchanged -- generic in lxRoot/lyRoot)
// ============================================================
void computeBalanceCompensation(float lxRoot, float lyRoot,
                                 float pitchRad, float rollRad,
                                 float gain, float levelBlend,
                                 float &txComp, float &tyComp, float &dz) {
    float lx = lxRoot - LEVEL_CENTER_X;
    float ly = lyRoot - LEVEL_CENTER_Y;

    float cp = cos(pitchRad), sp = sin(pitchRad);
    float cr = cos(rollRad),  sr = sin(rollRad);

    txComp = lx * (cp - 1.0f) * gain * levelBlend;
    tyComp = (lx * sp * sr + ly * (cr - 1.0f)) * gain * levelBlend;
    dz     = (lx * sp * cr - ly * sr) * gain * levelBlend;

    float reachMargin = currentReachMargin();
    float zClamp  = min(LEVEL_MAX_DELTA_Z, reachMargin);
    float xyClamp = min(LEVEL_MAX_DELTA_XY, reachMargin);

    txComp = constrain(txComp, -xyClamp, xyClamp);
    tyComp = constrain(tyComp, -xyClamp, xyClamp);
    dz     = constrain(dz, -zClamp, zClamp);
}

// ============================================================
// PUPPET MODE GEOMETRY (unchanged -- generic in lxRoot/lyRoot)
// ============================================================
float softDeadband(float v, float threshold) {
    if (fabsf(v) <= threshold) return 0.0f;
    return (v > 0.0f) ? (v - threshold) : (v + threshold);
}

struct OneEuro {
    float xPrev = 0.0f, dxPrev = 0.0f;
    bool  primed = false;
};

float oneEuroAlpha(float cutoffHz, float dt) {
    float tau = 1.0f / (2.0f * PI * cutoffHz);
    return 1.0f / (1.0f + tau / dt);
}

float oneEuroFilter(OneEuro &st, float x, float dt) {
    if (dt <= 0.0f) return st.primed ? st.xPrev : x;
    if (!st.primed) {
        st.primed = true;
        st.xPrev  = x;
        st.dxPrev = 0.0f;
        return x;
    }
    float dx = (x - st.xPrev) / dt;
    float aD = oneEuroAlpha(PUPPET_FILT_DCUTOFF, dt);
    float dxHat = aD * dx + (1.0f - aD) * st.dxPrev;
    st.dxPrev = dxHat;

    float cutoff = PUPPET_FILT_MINCUTOFF + PUPPET_FILT_BETA * fabsf(dxHat);
    float aX = oneEuroAlpha(cutoff, dt);
    float xHat = aX * x + (1.0f - aX) * st.xPrev;
    st.xPrev = xHat;
    return xHat;
}

float slewLimit(float current, float target, float maxRatePerSec, float dt) {
    float maxStep = maxRatePerSec * dt;
    float delta = target - current;
    if (delta >  maxStep) return current + maxStep;
    if (delta < -maxStep) return current - maxStep;
    return target;
}

void computePuppetFootOffsets(float lxRoot, float lyRoot,
                               float rollRad, float pitchRad, float yawRad,
                               float &dx, float &dy, float &dtz) {
    float lx = lxRoot - LEVEL_CENTER_X;
    float ly = lyRoot - LEVEL_CENTER_Y;

    float cr = cosf(rollRad),  sr = sinf(rollRad);
    float cp = cosf(pitchRad), sp = sinf(pitchRad);
    float cy = cosf(yawRad),   sy = sinf(yawRad);

    float fx = ( cp * cy)              * lx + ( cp * sy)              * ly;
    float fy = (sr * sp * cy - cr * sy) * lx + (sr * sp * sy + cr * cy) * ly;
    float fz = (cr * sp * cy + sr * sy) * lx + (cr * sp * sy - sr * cy) * ly;

    float reachMargin = currentReachMargin();
    float zClamp  = min(PUPPET_MAX_DELTA_Z,  reachMargin);
    float xyClamp = min(PUPPET_MAX_DELTA_XY, reachMargin);

    dx  = constrain(fx - lx, -xyClamp, xyClamp);
    dy  = constrain(fy - ly, -xyClamp, xyClamp);
    dtz = constrain(-fz,     -zClamp,  zClamp);
}

// ============================================================
// IDLE "ALIVE" MOTION (unchanged)
// ============================================================
struct IdleAxis { float ou = 0.0f, smooth = 0.0f; };
IdleAxis idleRoll, idlePitch, idleHeight;
float idleBreathPhase = 0.0f;

float idleGauss() {
    float u1 = (esp_random() + 1.0f) / 4294967296.0f;
    float u2 = esp_random() / 4294967296.0f;
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI * u2);
}

float idleVarianceComp() {
    return sqrtf((IDLE_DRIFT_TAU_S + IDLE_SMOOTH_TAU_S) / IDLE_DRIFT_TAU_S);
}

float idleStepAxis(IdleAxis &ax, float a, float noiseGain, float b, float comp) {
    ax.ou     = a * ax.ou + noiseGain * idleGauss();
    ax.smooth = b * ax.smooth + (1.0f - b) * ax.ou;
    return ax.smooth * comp;
}

float idleBreathWave(float phase) {
    float f = constrain(IDLE_BREATH_INHALE_FRAC, 0.05f, 0.95f);
    float shaped = (phase < f) ? (0.5f * phase / f)
                               : (0.5f + 0.5f * (phase - f) / (1.0f - f));
    return -cosf(2.0f * PI * shaped);
}

float idleClamp(float v, float limit) { return constrain(v, -limit, limit); }

void idleReset() {
    idleRoll = idlePitch = idleHeight = IdleAxis();
    idle_roll_rad = idle_pitch_rad = idle_dz_m = 0.0f;
    idle_envelope = 0.0f;
}

void idleUpdate(float dt, float activity, bool allowed) {
    if (dt <= 0.0f) dt = 1e-4f;

    float targetEnv = allowed ? (1.0f - constrain(activity, 0.0f, 1.0f)) : 0.0f;
    float tau = (targetEnv > idle_envelope) ? IDLE_FADE_IN_S : IDLE_FADE_OUT_S;
    idle_envelope += (targetEnv - idle_envelope) * (1.0f - expf(-dt / max(tau, 1e-3f)));

    idleBreathPhase = fmodf(idleBreathPhase + dt / IDLE_BREATH_PERIOD_S, 1.0f);
    float breath = idleBreathWave(idleBreathPhase);

    float a = expf(-dt / IDLE_DRIFT_TAU_S);
    float noiseGain = sqrtf(max(0.0f, 1.0f - a * a));
    float b = expf(-dt / IDLE_SMOOTH_TAU_S);
    float comp = idleVarianceComp();

    float dRoll   = idleStepAxis(idleRoll,   a, noiseGain, b, comp);
    float dPitch  = idleStepAxis(idlePitch,  a, noiseGain, b, comp);
    float dHeight = idleStepAxis(idleHeight, a, noiseGain, b, comp);

    float rollDeg  = dRoll  * IDLE_DRIFT_ROLL_DEG;
    float pitchDeg = dPitch * IDLE_DRIFT_PITCH_DEG + breath * IDLE_BREATH_PITCH_DEG;
    float dz       = dHeight * IDLE_DRIFT_HEIGHT_M + breath * IDLE_BREATH_HEIGHT_M;

    idle_roll_rad  = radians(idleClamp(rollDeg,  IDLE_MAX_ROLL_DEG))  * idle_envelope;
    idle_pitch_rad = radians(idleClamp(pitchDeg, IDLE_MAX_PITCH_DEG)) * idle_envelope;
    idle_dz_m      = idleClamp(dz, IDLE_MAX_HEIGHT_M) * idle_envelope;
}

// ============================================================
// MANUAL PITCH TRIM 
// ============================================================
void computeManualPitchTilt(float lxRoot, float lyRoot, float trimRad,
                             float usedZ, float usedXY,
                             float &txComp, float &dz) {
    if (fabs(trimRad) < 1e-6f) { txComp = 0.0f; dz = 0.0f; return; }

    float cp = cos(trimRad), sp = sin(trimRad);
    txComp = lxRoot * (cp - 1.0f);
    dz     = lxRoot * sp;

    float reachMargin = currentReachMargin();
    float xyBudget = min(LEVEL_MAX_DELTA_XY, reachMargin);
    float zBudget  = min(LEVEL_MAX_DELTA_Z, reachMargin);
    float xyClamp = max(0.0f, xyBudget - fabs(usedXY));
    float zClamp  = max(0.0f, zBudget  - fabs(usedZ));

    txComp = constrain(txComp, -xyClamp, xyClamp);
    dz     = constrain(dz, -zClamp, zClamp);
}

// ============================================================
// BOW/TILT AUTO-CALIBRATION DANCE
// ============================================================
int   calState = 0;
float calTInState = 0.0f;

float calZeroSumAx = 0, calZeroSumAy = 0, calZeroSumAz = 0; int calZeroN = 0;
float calZeroGyroSum[3] = {0, 0, 0};
float calBaseline[4]    = {0, 0, 0, 0};
float calGyroBiasArr[3] = {0, 0, 0};

float calPoseSumAx = 0, calPoseSumAy = 0, calPoseSumAz = 0; int calPoseN = 0;
float calGyroIntG[3] = {0, 0, 0};

int   calPitchAccelKind = -1; float calPitchAccelSign = 1.0f;
int   calPitchGyroAxis  = -1; float calPitchGyroSign  = 1.0f;

bool  calibDoneFlag = false;
char  calStatusMsgBuf[48] = "not calibrated";

const int CAL_PITCH_CANDIDATES[2] = {0, 1};
const int CAL_ROLL_CANDIDATES[2]  = {2, 3};

bool calDone() { return calibDoneFlag; }
const char* calStatusMsg() { return calStatusMsgBuf; }
bool calRunnerIsRunning() { return calState != 0 && calState != 8; }

void calApplyCacheFallback() {
    if (!imuCalCacheLoaded) {
        Serial.println("[Cal] Dance FAILED and no cached calibration exists -- "
                       "BALANCE WILL STAY OFF this session. Re-run calibration on level ground.");
        return;
    }
    imuAccelPitchKind = imuCalCache.pKind;  imuAccelPitchSign = imuCalCache.pSign;
    imuAccelRollKind  = imuCalCache.rKind;  imuAccelRollSign  = imuCalCache.rSign;
    imuGyroPitchAxis  = imuCalCache.pGAxis; imuGyroPitchSign  = imuCalCache.pGSign;
    imuGyroRollAxis   = imuCalCache.rGAxis; imuGyroRollSign   = imuCalCache.rGSign;
    imuPitchBias      = imuCalCache.pBias;  imuRollBias       = imuCalCache.rBias;
    imuGyroBiasPitch  = imuCalCache.gBiasP; imuGyroBiasRoll   = imuCalCache.gBiasR;

    body_pitch_filtered = 0.0f;
    body_roll_filtered  = 0.0f;
    resetKalmanState(kalmanPitch);
    resetKalmanState(kalmanRoll);

    calibDoneFlag = true;
    strncpy(calStatusMsgBuf, "FAIL -> using cached cal", sizeof(calStatusMsgBuf));
    Serial.println("[Cal] Dance FAILED -- restored cached calibration from Flash. "
                   "Balance enabled on last known-good values.");
}

void calRunnerStart() {
    Serial.println("[Cal] Starting motion-based calibration...");
    calState = 1;
    calTInState = 0.0f;
    calZeroSumAx = calZeroSumAy = calZeroSumAz = 0.0f;
    calZeroN = 0;
    for (int i = 0; i < 3; i++) calZeroGyroSum[i] = 0.0f;
}

float calRampFn(float t, float dur) {
    float u = constrain(t / max(1e-6f, dur), 0.0f, 1.0f);
    return 0.5f - 0.5f * cos(PI * u);
}

float calRunnerLegDz(float lxRoot, float lyRoot) {
    float A = CAL_TILT_MAGNITUDE_M;
    switch (calState) {
        case 2: {
            float r = calRampFn(calTInState, CAL_BOW_DURATION_S);
            return (fabs(lxRoot) > 1e-6f) ? copysignf(A, lxRoot) * r : 0.0f;
        }
        case 3:
            return (fabs(lxRoot) > 1e-6f) ? copysignf(A, lxRoot) : 0.0f;
        case 4: {
            float r = 1.0f - calRampFn(calTInState, CAL_RETURN_DURATION_S);
            return (fabs(lxRoot) > 1e-6f) ? copysignf(A, lxRoot) * r : 0.0f;
        }
        case 5: {
            float r = calRampFn(calTInState, CAL_BOW_DURATION_S);
            return (fabs(lyRoot) > 1e-6f) ? copysignf(A, lyRoot) * r : 0.0f;
        }
        case 6:
            return (fabs(lyRoot) > 1e-6f) ? copysignf(A, lyRoot) : 0.0f;
        case 7: {
            float r = 1.0f - calRampFn(calTInState, CAL_RETURN_DURATION_S);
            return (fabs(lyRoot) > 1e-6f) ? copysignf(A, lyRoot) * r : 0.0f;
        }
        default:
            return 0.0f;
    }
}

void calResetMotionAccumulators() {
    calPoseSumAx = calPoseSumAy = calPoseSumAz = 0.0f;
    calPoseN = 0;
    calGyroIntG[0] = calGyroIntG[1] = calGyroIntG[2] = 0.0f;
}

void calGoState(int next) {
    calState = next;
    calTInState = 0.0f;
}

void calPickAccel(const int *candidates, int nCandidates, float baselineExpectedSign,
                   int &outKind, float &outSign, float &outSignedDelta) {
    float ax = calPoseSumAx / max(1, calPoseN);
    float ay = calPoseSumAy / max(1, calPoseN);
    float az = calPoseSumAz / max(1, calPoseN);

    int bestKind = candidates[0];
    float bestDelta = 0.0f;
    for (int i = 0; i < nCandidates; i++) {
        int k = candidates[i];
        float now = computeAccelAngle(k, 1.0f, ax, ay, az);
        float delta = now - calBaseline[k];

        Serial.printf("[CalDbg] cand=%d now=%+.2fdeg base=%+.2fdeg delta=%+.2fdeg (need >=%.2fdeg)\n",
                      k, degrees(now), degrees(calBaseline[k]), degrees(delta), degrees(CAL_MIN_DETECT_RAD));
        if (fabs(delta) > fabs(bestDelta)) { bestDelta = delta; bestKind = k; }
    }
    Serial.printf("[CalDbg] accel avg=(%+.2f,%+.2f,%+.2f) bestKind=%d bestDelta=%+.2fdeg\n",
                  ax, ay, az, bestKind, degrees(bestDelta));

    if (fabs(bestDelta) < CAL_MIN_DETECT_RAD) { outKind = -1; outSign = 1.0f; outSignedDelta = 0.0f; return; }

    outSign = ((bestDelta * baselineExpectedSign) > 0.0f) ? 1.0f : -1.0f;
    outKind = bestKind;
    outSignedDelta = bestDelta * outSign;
}

void calPickGyro(float targetSign, int excludeAxis, int &outAxis, float &outSign) {
    int bestAxis = -1; float bestAbs = -1.0f;

    Serial.printf("[CalDbg] gyroInt=(%+.2f,%+.2f,%+.2f)deg exclude=%d (need >=%.2fdeg)\n",
                  degrees(calGyroIntG[0]), degrees(calGyroIntG[1]), degrees(calGyroIntG[2]),
                  excludeAxis, degrees(CAL_MIN_GYRO_INT_RAD));
    for (int i = 0; i < 3; i++) {
        if (i == excludeAxis) continue;
        float v = fabs(calGyroIntG[i]);
        if (v > bestAbs) { bestAbs = v; bestAxis = i; }
    }
    if (bestAxis < 0 || bestAbs < CAL_MIN_GYRO_INT_RAD) { outAxis = -1; outSign = 1.0f; return; }
    outSign = ((calGyroIntG[bestAxis] * targetSign) > 0.0f) ? 1.0f : -1.0f;
    outAxis = bestAxis;
}

void calApplyRemap(int rollAccelKind, float rollAccelSign, int rollGyroAxis, float rollGyroSign) {
    imuAccelPitchKind = calPitchAccelKind;
    imuAccelPitchSign = calPitchAccelSign;
    imuAccelRollKind  = rollAccelKind;
    imuAccelRollSign  = rollAccelSign;
    imuGyroPitchAxis  = calPitchGyroAxis;
    imuGyroPitchSign  = calPitchGyroSign;
    imuGyroRollAxis   = rollGyroAxis;
    imuGyroRollSign   = rollGyroSign;

    imuPitchBias = calBaseline[imuAccelPitchKind] * imuAccelPitchSign;
    imuRollBias  = calBaseline[imuAccelRollKind]  * imuAccelRollSign;

    imuGyroBiasPitch = calGyroBiasArr[imuGyroPitchAxis] * imuGyroPitchSign;
    imuGyroBiasRoll  = calGyroBiasArr[imuGyroRollAxis]  * imuGyroRollSign;

    body_pitch_filtered = 0.0f;
    body_roll_filtered  = 0.0f;
    resetKalmanState(kalmanPitch);
    resetKalmanState(kalmanRoll);

    calibDoneFlag = true;
    strncpy(calStatusMsgBuf, "OK", sizeof(calStatusMsgBuf));

    Serial.println("[Cal] Complete. IMU remap:");
    Serial.printf("      pitch_accel kind=%d sign=%+.0f | roll_accel kind=%d sign=%+.0f\n",
                  imuAccelPitchKind, imuAccelPitchSign, imuAccelRollKind, imuAccelRollSign);
    Serial.printf("      pitch_gyro axis=%d sign=%+.0f | roll_gyro axis=%d sign=%+.0f\n",
                  imuGyroPitchAxis, imuGyroPitchSign, imuGyroRollAxis, imuGyroRollSign);
    Serial.printf("      pitch_bias=%+.3fdeg roll_bias=%+.3fdeg gyroBiasP=%+.4fdeg/s gyroBiasR=%+.4fdeg/s\n",
                  degrees(imuPitchBias), degrees(imuRollBias),
                  degrees(imuGyroBiasPitch), degrees(imuGyroBiasRoll));
    Serial.printf("      Kalman gains left at fixed defaults: Q_ANGLE=%.4f Q_BIAS=%.4f R_MEASURE=%.4f\n",
                  KALMAN_Q_ANGLE, KALMAN_Q_BIAS, KALMAN_R_MEASURE);

    saveImuCalToFlash();
}

void calRunnerStep(float dt) {
    if (!calRunnerIsRunning()) return;
    calTInState += dt;

    switch (calState) {
        case 1: {
            calZeroSumAx += imu_ax; calZeroSumAy += imu_ay; calZeroSumAz += imu_az;
            calZeroN++;
            float gvals[3] = {raw_gx, raw_gy, raw_gz};
            for (int i = 0; i < 3; i++) calZeroGyroSum[i] += gvals[i];
            if (calTInState >= CAL_ZERO_DURATION_S) {
                int n = max(1, calZeroN);
                float ax = calZeroSumAx / n, ay = calZeroSumAy / n, az = calZeroSumAz / n;
                for (int i = 0; i < 4; i++) calBaseline[i] = computeAccelAngle(i, 1.0f, ax, ay, az);
                for (int i = 0; i < 3; i++) calGyroBiasArr[i] = calZeroGyroSum[i] / n;
                Serial.printf("[Cal] Zero done. accel=(%+.2f,%+.2f,%+.2f) gyroBias=(%+.3f,%+.3f,%+.3f)deg/s\n",
                              ax, ay, az, degrees(calGyroBiasArr[0]), degrees(calGyroBiasArr[1]), degrees(calGyroBiasArr[2]));
                calResetMotionAccumulators();
                calGoState(2);
            }
            break;
        }
        case 2: {
            calGyroIntG[0] += raw_gx * dt; calGyroIntG[1] += raw_gy * dt; calGyroIntG[2] += raw_gz * dt;
            if (calTInState >= CAL_BOW_DURATION_S) {
                calPoseSumAx = calPoseSumAy = calPoseSumAz = 0.0f; calPoseN = 0;
                calGoState(3);
            }
            break;
        }
        case 3: {
            calPoseSumAx += imu_ax; calPoseSumAy += imu_ay; calPoseSumAz += imu_az; calPoseN++;
            if (calTInState >= CAL_HOLD_DURATION_S) {
                int kind; float sgn, signedDelta;
                calPickAccel(CAL_PITCH_CANDIDATES, 2, -1.0f, kind, sgn, signedDelta);
                if (kind < 0) {
                    strncpy(calStatusMsgBuf, "FAIL: bow accel response too small", sizeof(calStatusMsgBuf));
                    Serial.println(calStatusMsgBuf);
                    calApplyCacheFallback(); calGoState(8); break;
                }
                int gaxis; float gsgn;
                calPickGyro(signedDelta, -1, gaxis, gsgn);
                if (gaxis < 0) {
                    strncpy(calStatusMsgBuf, "FAIL: bow gyro response too small", sizeof(calStatusMsgBuf));
                    Serial.println(calStatusMsgBuf);
                    calApplyCacheFallback(); calGoState(8); break;
                }
                calPitchAccelKind = kind; calPitchAccelSign = sgn;
                calPitchGyroAxis  = gaxis; calPitchGyroSign  = gsgn;
                Serial.printf("[Cal] Pitch: accelKind=%d*%+.0f gyroAxis=%d*%+.0f (delta=%+.2fdeg)\n",
                              kind, sgn, gaxis, gsgn, degrees(signedDelta));
                calGoState(4);
            }
            break;
        }
        case 4:
            if (calTInState >= CAL_RETURN_DURATION_S) {
                calResetMotionAccumulators();
                calGoState(5);
            }
            break;
        case 5: {
            calGyroIntG[0] += raw_gx * dt; calGyroIntG[1] += raw_gy * dt; calGyroIntG[2] += raw_gz * dt;
            if (calTInState >= CAL_BOW_DURATION_S) {
                calPoseSumAx = calPoseSumAy = calPoseSumAz = 0.0f; calPoseN = 0;
                calGoState(6);
            }
            break;
        }
        case 6: {
            calPoseSumAx += imu_ax; calPoseSumAy += imu_ay; calPoseSumAz += imu_az; calPoseN++;
            if (calTInState >= CAL_HOLD_DURATION_S) {
                int kind; float sgn, signedDelta;
                calPickAccel(CAL_ROLL_CANDIDATES, 2, 1.0f, kind, sgn, signedDelta);
                if (kind < 0) {
                    strncpy(calStatusMsgBuf, "FAIL: tilt accel response too small", sizeof(calStatusMsgBuf));
                    Serial.println(calStatusMsgBuf);
                    calApplyCacheFallback(); calGoState(8); break;
                }
                int gaxis; float gsgn;
                calPickGyro(signedDelta, -1, gaxis, gsgn);
                if (gaxis < 0) {
                    strncpy(calStatusMsgBuf, "FAIL: tilt gyro response too small", sizeof(calStatusMsgBuf));
                    Serial.println(calStatusMsgBuf);
                    calApplyCacheFallback(); calGoState(8); break;
                }
                if (gaxis == calPitchGyroAxis) {
                    Serial.println("[Cal] WARNING: roll gyro axis same as pitch -- picking next-best.");
                    calPickGyro(signedDelta, calPitchGyroAxis, gaxis, gsgn);
                    if (gaxis < 0) { gaxis = (calPitchGyroAxis + 1) % 3; gsgn = 1.0f; }
                }
                Serial.printf("[Cal] Roll: accelKind=%d*%+.0f gyroAxis=%d*%+.0f (delta=%+.2fdeg)\n",
                              kind, sgn, gaxis, gsgn, degrees(signedDelta));
                calApplyRemap(kind, sgn, gaxis, gsgn);
                calGoState(7);
            }
            break;
        }
        case 7:
            if (calTInState >= CAL_RETURN_DURATION_S) calGoState(8);
            break;
        default:
            break;
    }
}

// ============================================================
// INVERSE KINEMATICS
// ============================================================
bool solve_leg_ik_3dof(float tx, float ty, float tz, float urdf_x_offset, float &hip, float &thigh, float &knee) {
    float x = tx + urdf_x_offset;
    float y = ty;
    float z_from_thigh = -(tz - BODY_OFFSET);

    hip = atan2(y, -z_from_thigh);
    float z_sag = -sqrt(y*y + z_from_thigh*z_from_thigh);

    float dist_sq = x*x + z_sag*z_sag;
    float dist = sqrt(dist_sq);

    if (dist > (L1 + L2) * 0.99 || dist < abs(L1 - L2)) return false;

    float cos_phi = (L1*L1 + L2*L2 - dist_sq) / (2.0 * L1 * L2);
    knee = PI - acos(constrain(cos_phi, -1.0, 1.0));

    float alpha = atan2(z_sag, x);
    float cos_beta = (L1*L1 + dist_sq - L2*L2) / (2.0 * L1 * dist);

    float beta = acos(constrain(cos_beta, -1.0, 1.0));
    thigh = alpha - beta + PI/2.0;
    return true;
}

// ============================================================
// GAIT FOOT-TRAVEL PROFILES
// ============================================================
float quintic_ease(float s) {
    return s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
}

float cycloid_ease(float s) {
    return s - sin(2.0f * PI * s) / (2.0f * PI);
}


void legProfiles(float phase, float duty, float &pm_q, float &pm_c, float &lift_q, float &lift_c) {
    float swing_frac = 1.0f - duty;
    if (phase < swing_frac) {
        float s = phase / swing_frac;
        float q = quintic_ease(s);
        pm_q = 1.0f - 2.0f * q;
        pm_c = 1.0f - 2.0f * cycloid_ease(s);
        lift_q = 4.0f * q * (1.0f - q);
        lift_c = sin(PI * s);
        return;
    }
    float s = (phase - swing_frac) / duty;
    pm_q = -1.0f + 2.0f * s;
    pm_c = -1.0f + 2.0f * cycloid_ease(s);
    lift_q = 0.0f;
    lift_c = 0.0f;
}

// ============================================================
// GAIT ROW SELECTION
// ============================================================
QuadGaitRow selectGaitRow(const char* gaitName, float t_norm) {
    float min_v = 999.0f, max_v = -999.0f;
    int matches[QUAD_GAIT_MATRIX_SIZE], match_count = 0;

    for (int i = 0; i < QUAD_GAIT_MATRIX_SIZE; i++) {
        if (strcmp(quadGaitMatrix[i].gait, gaitName) == 0) {
            matches[match_count++] = i;
            if (quadGaitMatrix[i].target_vel < min_v) min_v = quadGaitMatrix[i].target_vel;
            if (quadGaitMatrix[i].target_vel > max_v) max_v = quadGaitMatrix[i].target_vel;
        }
    }
    if (match_count == 0) return quadGaitMatrix[0];   // fallback

    float target = min_v + t_norm * (max_v - min_v);
    int best_idx = matches[0];
    float min_err = fabs(quadGaitMatrix[best_idx].target_vel - target);
    for (int i = 1; i < match_count; i++) {
        int idx = matches[i];
        float err = fabs(quadGaitMatrix[idx].target_vel - target);
        if (err < min_err) { min_err = err; best_idx = idx; }
    }
    return quadGaitMatrix[best_idx];
}

// ============================================================
// GAIT COMMAND MIXING
// ============================================================
struct GaitCmd {
    float freq, duty, amp_t, amp_r, sway, straight_mix;
    float c, s, spin_dir;
    bool  timing_is_walk;
};

GaitCmd computeGaitCommand(float fwd, float side, float spin) {
    float t_raw = sqrt(fwd * fwd + side * side);
    float t = min(1.0f, t_raw);
    float r = fabs(spin);
    float m_max = max(max(t, r), 1e-9f);

    float tw = (t > 0.001f) ? t / m_max : 0.0f;
    float rw = (r > 0.001f) ? r / m_max : 0.0f;
    float c, s;
    if (t_raw > 1e-6f) { c = fwd / t_raw; s = side / t_raw; }
    else               { c = 1.0f; s = 0.0f; }

    float fa = fabs(c), sa = fabs(s);
    float mn = min(fa, sa);
    float tot = max(fa + sa, 1e-9f);
    float w_str = (fa - mn) / tot, w_sid = (sa - mn) / tot, w_dia = 2.0f * mn / tot;

    float denom = max(tw + rw, 1e-9f);

    const char* straightGait = walk_gait_enabled ? "walk" : "trot";
    bool use_walk = walk_gait_enabled;

    float ft = 0, speed_t = 0, duty_t = 0, sway_t = 0;
    if (tw > 0.0f) {
        QuadGaitRow r_str = selectGaitRow(straightGait, t);
        QuadGaitRow r_sid = selectGaitRow("sideway", t);
        QuadGaitRow r_dia = selectGaitRow("diagonal", t);
        float straight_duty = use_walk ? r_str.duty : TROT_DUTY;

        float ws[3]    = { w_str,          w_sid,               w_dia };
        float freqs[3] = { r_str.frequency, r_sid.frequency,     r_dia.frequency };
        float amps[3]  = { r_str.step_amplitude, r_sid.step_amplitude, r_dia.step_amplitude };
        float duts[3]  = { straight_duty,  SIDEWAY_DEFAULT_DUTY, r_dia.duty };

        for (int i = 0; i < 3; i++) {
            ft      += ws[i] * freqs[i];
            speed_t += ws[i] * amps[i] * freqs[i];
            duty_t  += ws[i] * duts[i];
        }
        sway_t = w_str * r_str.sway_amp;
    }

    float fr = 0, ar = 0;
    if (rw > 0.0f) {
        QuadGaitRow r_spi = selectGaitRow("spin", r);
        fr = r_spi.frequency; ar = r_spi.step_amplitude;
    }

    float freq = (tw * ft + rw * fr) / denom;
    freq = max(freq, 1e-3f);
    float duty = (tw * duty_t + rw * 0.5f) / denom;

    GaitCmd cmd;
    cmd.freq = freq;
    cmd.duty = duty;
    cmd.amp_t = tw * speed_t / freq;
    cmd.amp_r = rw * ar * fr / freq;
    cmd.sway = tw * sway_t;
    cmd.straight_mix = (tw > 0.0f) ? w_str : 0.0f;
    cmd.c = c; cmd.s = s;
    cmd.spin_dir = (spin > 0.0f) ? -1.0f : 1.0f;
    cmd.timing_is_walk = use_walk;
    return cmd;
}

// ============================================================
// CONTROL INPUT
// ============================================================
ControlInput parseUdpPacket(const uint8_t* buf, int packetSize) {
    ControlInput in;
    memcpy(&in.fwd,  &buf[0],  4);
    memcpy(&in.side, &buf[4],  4);
    memcpy(&in.spin, &buf[8],  4);
    memcpy(&in.lt,   &buf[12], 4);
    memcpy(&in.rt,   &buf[16], 4);
    in.buttons1 = buf[20];
    in.emoteId  = buf[21];
    in.extended = (packetSize >= RX_PACKET_EXT_SIZE);
    if (in.extended) {
        in.buttons2 = buf[22];
        for (int i = 0; i < 6; i++) memcpy(&in.tunables[i], &buf[24 + i * 4], 4);
    }
    if (packetSize >= RX_PACKET_PUP_SIZE) {
        in.puppetValid  = true;
        uint8_t pf      = buf[48];
        in.puppetOn     = (pf & 0x01) != 0;
        in.puppetRezero = (pf & 0x02) != 0;
        memcpy(&in.puppetRoll,  &buf[49], 4);
        memcpy(&in.puppetPitch, &buf[53], 4);
        memcpy(&in.puppetGz,    &buf[57], 4);
    }
    return in;
}

void processControlInput(const ControlInput& in) {
    static uint32_t lastInputMicros = 0;
    uint32_t nowMicros = micros();
    float in_dt = (lastInputMicros == 0) ? DT_SEC
                                         : (nowMicros - lastInputMicros) / 1000000.0f;
    lastInputMicros = nowMicros;
    in_dt = constrain(in_dt, 0.0f, 0.1f);

    float fwd = in.fwd, side = in.side, spin = in.spin, lt = in.lt, rt = in.rt;
    uint8_t buttons        = in.buttons1;
    uint8_t incoming_emote = in.emoteId;

    bool buttonAPressed          = (buttons & 0x01) != 0;
    bool buttonBPressed          = (buttons & 0x02) != 0;
    bool buttonL1Pressed         = (buttons & 0x04) != 0;
    bool buttonR1Pressed         = (buttons & 0x08) != 0;
    bool emoteModeTogglePressed  = (buttons & 0x10) != 0;
    bool emotePlayPressed        = (buttons & 0x20) != 0;
    bool emoteStopPressed        = (buttons & 0x40) != 0;

    bool l2Pressed = false;
    bool r2Pressed = false;

    if (in.extended) {
        uint8_t buttons2 = in.buttons2;
        bool manualCalTrigger = (buttons2 & 0x01) != 0;
        kill_switch_active    = (buttons2 & 0x02) != 0;
        bool tunablesValid    = (buttons2 & 0x08) != 0;
        l2Pressed             = (buttons2 & 0x40) != 0;
        r2Pressed             = (buttons2 & 0x80) != 0;

        if (tunablesValid) {
            LEVEL_GAIN         = in.tunables[0];
            KALMAN_Q_ANGLE     = in.tunables[1];
            KALMAN_Q_BIAS      = in.tunables[2];
            KALMAN_R_MEASURE   = in.tunables[3];
            LEVEL_DEADBAND_DEG = in.tunables[4];
            LEVEL_MAX_TILT_DEG = in.tunables[5];
        }

        if (manualCalTrigger && !lastManualCalTriggerPressed) {
            bool isFullyActiveNow = (current_transition_progress >= 1.0f) && !in_calibration_mode;
            if (isFullyActiveNow && !emote_mode_enabled && !calRunnerIsRunning()) {
                calRunnerStart();
            }
        }
        lastManualCalTriggerPressed = manualCalTrigger;
    }

    static uint8_t last_incoming_emote = 0xFF;
    if (incoming_emote != last_incoming_emote) {
        if (incoming_emote < NUM_EMOTES) selected_emote_id = incoming_emote;
        last_incoming_emote = incoming_emote;
    }

    float f_in = (abs(fwd)  > JOYSTICK_DEADZONE) ? fwd  : 0.0f;
    float s_in = (abs(side) > JOYSTICK_DEADZONE) ? side : 0.0f;

    if (f_in != 0.0f || s_in != 0.0f) {
        static bool snappedToFwd = false, snappedToSide = false;
        float angDeg = degrees(atan2f(fabsf(s_in), fabsf(f_in)));
        float snapIn  = AXIS_SNAP_DEG;
        float snapOut = AXIS_SNAP_DEG + AXIS_SNAP_HYST_DEG;

        snappedToFwd  = snappedToFwd  ? (angDeg <  snapOut)        : (angDeg <  snapIn);
        snappedToSide = snappedToSide ? (angDeg > (90.0f - snapOut)) : (angDeg > (90.0f - snapIn));

        if (snappedToFwd)  s_in = 0.0f;
        else if (snappedToSide) f_in = 0.0f;
    }

    if (fabsf(f_in) > 0.0f || fabsf(s_in) > 0.0f ||
        (fabsf(spin) > JOYSTICK_DEADZONE) ||
        lt > TRIGGER_THRESHOLD || rt > TRIGGER_THRESHOLD ||
        buttons != 0 || (in.extended && in.buttons2 != 0)) {
        last_activity_ms = millis();
    }

    joy_fwd  = f_in;
    joy_side = s_in;
    joy_spin = (abs(spin) > JOYSTICK_DEADZONE) ? spin : 0.0;
    norm_lt  = lt;
    norm_rt  = rt;

    bool is_mid_transition = (target_standing_state && current_transition_progress < 1.0) ||
                             (!target_standing_state && current_transition_progress > 0.0);

    if (buttonL1Pressed && buttonR1Pressed && !is_mid_transition
        && !in_calibration_mode && !emote_mode_enabled) {
        button_hold_time += in_dt;
        if (button_hold_time >= REQUIRED_HOLD_DURATION) {
            target_standing_state = !target_standing_state;
            button_hold_time = 0.0;
        }
    } else {
        button_hold_time = 0.0;
    }

    bool fully_sitting = (!target_standing_state) && (current_transition_progress <= 0.0);
    bool menuInputAllowed = fully_sitting || in_calibration_mode || emote_mode_enabled;

    static bool shoulderChordSeen = false;
    if (buttonL1Pressed && buttonR1Pressed) shoulderChordSeen = true;
    bool navUp   = !buttonL1Pressed && lastL1Pressed && !shoulderChordSeen;
    bool navDown = !buttonR1Pressed && lastR1Pressed && !shoulderChordSeen;
    if (!buttonL1Pressed && !buttonR1Pressed) shoulderChordSeen = false;

    bool l1Busy = buttonL1Pressed || lastL1Pressed;
    bool r1Busy = buttonR1Pressed || lastR1Pressed;
    bool l2Sig = (l2Pressed || lt > 0.6f) && !l1Busy;
    bool r2Sig = (r2Pressed || rt > 0.6f) && !r1Busy;
    static bool l2NavHeld = false, r2NavHeld = false;
    bool navLeft = false, navRight = false;
    if (!l2NavHeld && l2Sig)                          { l2NavHeld = true; navLeft = true; }
    else if (l2NavHeld && !l2Pressed && lt < 0.3f)    { l2NavHeld = false; }
    if (!r2NavHeld && r2Sig)                          { r2NavHeld = true; navRight = true; }
    else if (r2NavHeld && !r2Pressed && rt < 0.3f)    { r2NavHeld = false; }
    if (l1Busy && (l2Pressed || lt > 0.6f)) l2NavHeld = true;
    if (r1Busy && (r2Pressed || rt > 0.6f)) r2NavHeld = true;

    if (DEBUG_MENU_INPUT && menuInputAllowed) {
        static uint8_t lastDbgKey = 0xFF;
        uint8_t key = (buttonL1Pressed << 0) | (buttonR1Pressed << 1) | (l2Pressed << 2) |
                      (r2Pressed << 3) | ((lt > 0.6f) << 4) | ((rt > 0.6f) << 5);
        if (key != lastDbgKey) {
            lastDbgKey = key;
            Serial.printf("[MenuIn] L1:%d R1:%d L2bit:%d R2bit:%d lt:%.2f rt:%.2f\n",
                          buttonL1Pressed, buttonR1Pressed, l2Pressed, r2Pressed, lt, rt);
        }
    }

    if (menuInputAllowed && uiInputQueue != NULL) {
        uint8_t ev;
        if (navUp)    { ev = UI_UP;    xQueueSend(uiInputQueue, &ev, 0); }
        if (navDown)  { ev = UI_DOWN;  xQueueSend(uiInputQueue, &ev, 0); }
        if (navLeft)  { ev = UI_LEFT;  xQueueSend(uiInputQueue, &ev, 0); }
        if (navRight) { ev = UI_RIGHT; xQueueSend(uiInputQueue, &ev, 0); }
        if (buttonAPressed && !lastAPressed) { ev = UI_ENTER; xQueueSend(uiInputQueue, &ev, 0); }
        if (buttonBPressed && !lastBPressed) { ev = UI_BACK;  xQueueSend(uiInputQueue, &ev, 0); }
    }

    bool is_fully_active = (current_transition_progress >= 1.0) && !in_calibration_mode;

    if (is_fully_active && !emote_mode_enabled && !puppet_mode_enabled) {
        if (norm_rt > TRIGGER_THRESHOLD) user_selected_height += HEIGHT_SPEED * norm_rt * in_dt;
        if (norm_lt > TRIGGER_THRESHOLD) user_selected_height -= HEIGHT_SPEED * norm_lt * in_dt;
        user_selected_height = constrain(user_selected_height, MIN_HEIGHT, MAX_HEIGHT);

        if (buttonAPressed && !lastAPressed) balance_enabled    = !balance_enabled;
        if (buttonBPressed && !lastBPressed) walk_gait_enabled  = !walk_gait_enabled;

        float trimStep = radians(PITCH_TRIM_SPEED_DEG_S) * in_dt;
        if (!balance_enabled) {
            if (l2Pressed && !r2Pressed) {
                manual_pitch_trim -= trimStep;
            } else if (r2Pressed && !l2Pressed) {
                manual_pitch_trim += trimStep;
            }
            manual_pitch_trim = constrain(manual_pitch_trim,
                                           -radians(PITCH_TRIM_MAX_DEG),
                                           radians(PITCH_TRIM_MAX_DEG));
        } else {
            if (manual_pitch_trim > trimStep) {
                manual_pitch_trim -= trimStep;
            } else if (manual_pitch_trim < -trimStep) {
                manual_pitch_trim += trimStep;
            } else {
                manual_pitch_trim = 0.0f;
            }
        }
    } else if (!in_calibration_mode) {
        joy_fwd = joy_side = joy_spin = 0.0;
    }

    if (emoteModeTogglePressed && !lastEmoteModeTogglePressed) {
        if (is_fully_active && !emote_mode_enabled) {
            emote_mode_enabled = true;
            emote_playing      = false;
            emote_playing_id   = EMOTE_NONE;

            balance_enabled    = false;
            walk_gait_enabled  = false;

            exiting_emote_ramp = false;
        } else if (emote_mode_enabled) {
            emote_mode_enabled = false;
            emote_playing      = false;
            emote_playing_id   = EMOTE_NONE;

            exiting_emote_ramp = true;
        }
    }

    static float puppetZeroRoll = 0.0f, puppetZeroPitch = 0.0f;
    static bool  puppetZeroCaptured = false;
    static float puppetYawDeg = 0.0f;

    if (in.puppetValid) {
        puppet_last_rx_ms = millis();

        bool wantOn = in.puppetOn && is_fully_active && !emote_mode_enabled
                      && !in_calibration_mode;
        if (wantOn && !puppet_mode_enabled) {
            balance_enabled    = false;
            walk_gait_enabled  = false;
            manual_pitch_trim  = 0.0f;
            puppetZeroCaptured = false;
            puppetYawDeg       = 0.0f;
        }
        puppet_mode_enabled = wantOn;

        if (puppet_mode_enabled) {
            if (in.puppetRezero || !puppetZeroCaptured) {
                puppetZeroRoll  = in.puppetRoll;
                puppetZeroPitch = in.puppetPitch;
                puppetYawDeg    = 0.0f;
                puppetZeroCaptured = true;
            }

            float gz = in.puppetGz;
            if (fabsf(gz) < PUPPET_GZ_DEADBAND) gz = 0.0f;
            puppetYawDeg += gz * in_dt;
            puppetYawDeg *= expf(-in_dt / PUPPET_YAW_LEAK_TAU);

            static OneEuro filtRoll, filtPitch, filtYaw;
            float fr = oneEuroFilter(filtRoll,  in.puppetRoll  - puppetZeroRoll,  in_dt);
            float fp = oneEuroFilter(filtPitch, in.puppetPitch - puppetZeroPitch, in_dt);
            float fy = oneEuroFilter(filtYaw,   puppetYawDeg,                     in_dt);

            float rel_r = softDeadband(fr, PUPPET_DEADBAND_DEG) * PUPPET_SIGN_ROLL;
            float rel_p = softDeadband(fp, PUPPET_DEADBAND_DEG) * PUPPET_SIGN_PITCH;
            float rel_y = fy * PUPPET_SIGN_YAW;

            puppet_tgt_roll_deg  = constrain(rel_r * PUPPET_ROLL_GAIN,  -PUPPET_MAX_ROLL_DEG,  PUPPET_MAX_ROLL_DEG);
            puppet_tgt_pitch_deg = constrain(rel_p * PUPPET_PITCH_GAIN, -PUPPET_MAX_PITCH_DEG, PUPPET_MAX_PITCH_DEG);
            puppet_tgt_yaw_deg   = constrain(rel_y * PUPPET_YAW_GAIN,   -PUPPET_MAX_YAW_DEG,   PUPPET_MAX_YAW_DEG);
        } else {
            puppet_tgt_roll_deg = puppet_tgt_pitch_deg = puppet_tgt_yaw_deg = 0.0f;
            puppetZeroCaptured = false;
        }
    }

    static bool cal_menu_pending_stand = false;
    if (cal_menu_requested) {
        cal_menu_requested = false;
        if (!calRunnerIsRunning()) {
            target_standing_state  = true;
            cal_menu_pending_stand = true;
        }
    }
    if (cal_menu_pending_stand && current_transition_progress >= 1.0f
        && !in_calibration_mode && !emote_mode_enabled) {
        cal_menu_pending_stand = false;
        if (!calRunnerIsRunning()) calRunnerStart();
    }

    static bool emote_menu_pending_stand = false;
    if (emote_menu_requested) {
        emote_menu_requested = false;
        if (!emote_mode_enabled) {
            target_standing_state    = true;
            emote_menu_pending_stand = true;
        }
    }
    if (emote_menu_pending_stand && current_transition_progress >= 1.0f
        && !in_calibration_mode) {
        emote_menu_pending_stand = false;
        emote_mode_enabled = true;
        emote_playing      = false;
        emote_playing_id   = EMOTE_NONE;
        balance_enabled    = false;
        walk_gait_enabled  = false;
        exiting_emote_ramp = false;
    }

    if (emote_exit_request) {
        emote_exit_request = false;
        if (emote_mode_enabled) {
            emote_mode_enabled = false;
            emote_playing      = false;
            emote_playing_id   = EMOTE_NONE;
            exiting_emote_ramp = true;
        }
        emote_menu_pending_stand = false;
    }

    static bool pending_play = false;
    if (emotePlayPressed && !lastEmotePlayPressed && emote_mode_enabled && !emote_playing) {
        pending_play = true;
    }
    if (emote_play_request) {
        emote_play_request = false;
        if (emote_mode_enabled && !emote_playing) pending_play = true;
    }

    if (emoteStopPressed && !lastEmoteStopPressed && emote_mode_enabled) {
        emote_playing    = false;
        emote_playing_id = EMOTE_NONE;
        pending_play     = false;
    }

    if (pending_play && emote_mode_enabled && !emote_playing
        && abs(current_body_height - EMOTE_BODY_HEIGHT) < EMOTE_HEIGHT_SETTLED_M
        && selected_emote_id < NUM_EMOTES) {
        emote_playing_id = selected_emote_id;
        emote_start_ms   = millis();
        emote_playing    = true;
        pending_play     = false;
    }

    lastAPressed                = buttonAPressed;
    lastBPressed                = buttonBPressed;
    lastL1Pressed               = buttonL1Pressed;
    lastR1Pressed                = buttonR1Pressed;
    lastEmoteModeTogglePressed  = emoteModeTogglePressed;
    lastEmotePlayPressed        = emotePlayPressed;
    lastEmoteStopPressed        = emoteStopPressed;
}

// ============================================================
// WALKING PROCESS ENGINE LOOP (Core 1)
// ============================================================
void KinematicsTask(void * pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(8.333);

    float level_blend = 0.0f;
    float footingStableTimer = 0.0f;
    float startupSettleTimer = 0.0f;
    bool  autoCalAttempted   = false;

    // ---- Gait-engine state (new for the quadruped trot/walk mixer) ----
    GaitCmd sm;   // low-pass-filtered gait parameters
    sm.freq = 1.0f; sm.duty = TROT_DUTY; sm.amp_t = 0.0f; sm.amp_r = 0.0f;
    sm.sway = 0.0f; sm.straight_mix = 1.0f; sm.c = 1.0f; sm.s = 0.0f;
    sm.spin_dir = 1.0f; sm.timing_is_walk = false;
    bool timingIsWalk = false;
    const float* currentOffsets = TROT_OFFSETS;
    bool pending_blend_reset = false;
    float blend_offset[4][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
    float last_leg_target[4][3];
    bool  have_last_leg_target = false;
    const float PARAM_ALPHA = 1.0f - expf(-DT_SEC / PARAM_SMOOTH_TAU);
    const float BLEND_ALPHA = 1.0f - expf(-DT_SEC / GAIT_BLEND_TAU);

    for(;;) {
        updateIMUAndBalance();

        if (imu_zero_request) {
            imu_zero_request = false;
            imuUserPitchZero += body_pitch_filtered;
            imuUserRollZero  += body_roll_filtered;
            body_pitch_filtered = 0.0f;
            body_roll_filtered  = 0.0f;
            resetKalmanState(kalmanPitch);
            resetKalmanState(kalmanRoll);
            imu_zero_save_request = true;
        }

        int packetSize = udp.parsePacket();
        if (packetSize >= RX_PACKET_MIN_SIZE) {
            int readSize = min(packetSize, (int)sizeof(networkBuffer));
            udp.read(networkBuffer, readSize);
            if (bootSequenceComplete) {
                processControlInput(parseUdpPacket(networkBuffer, packetSize));
            }
        }

        {
            ControlInput btIn;
            if (btPoll(btIn) && bootSequenceComplete) processControlInput(btIn);
        }

        if (kill_switch_active) {
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                shutDownServosHardware();
                xSemaphoreGive(i2cMutex);
            }
            target_standing_state       = false;
            current_transition_progress = 0.0f;
            emote_mode_enabled           = false;
            in_calibration_mode          = false;
            balance_enabled               = false;
            calState = 0;
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        if (!bootSequenceComplete) {
            if (millis() - bootStartTime >= BOOT_DURATION_MS) bootSequenceComplete = true;
        }

        if (bootSequenceComplete) {
            if (in_calibration_mode) {
                servosArePowered = true;
                current_transition_progress = 0.0;
            } else if (target_standing_state || emote_mode_enabled) {
                servosArePowered = true;
                current_transition_progress = min(1.0f, current_transition_progress + (DT_SEC / STARTUP_DURATION));
            } else {
                current_transition_progress = max(0.0f, current_transition_progress - (DT_SEC / STARTUP_DURATION));
                if (current_transition_progress <= 0.0 && servosArePowered) {
                    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        shutDownServosHardware();
                        xSemaphoreGive(i2cMutex);
                    }
                }
            }
        }

        float smooth_progress = 0.5 - 0.5 * cos(PI * current_transition_progress);
        bool is_fully_active = (current_transition_progress >= 1.0) && !in_calibration_mode;

        if (!target_standing_state && !servosArePowered && !in_calibration_mode && !emote_mode_enabled) {
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // ----- CALIBRATION NEUTRAL POSE -----
        if (in_calibration_mode) {
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                for (int idx = 0; idx < 4; idx++) {
                    String legName = LEG_ORDER[idx];
                    bool is_left = legName.endsWith("L");
                    int hipHWID, thighHWID, kneeHWID;
                    legServoIds(legName, kneeHWID, thighHWID, hipHWID);
                    if (kneeHWID < 0) continue;

                    if (is_left) {
                        setServo(hipHWID,   90.0);
                        setServo(thighHWID, 90.0);
                        setServo(kneeHWID,  90.0);
                    } else {
                        setServo(hipHWID,   90.0);
                        setServo(thighHWID, 90.0);
                        setServo(kneeHWID,  90.0);
                    }
                }
                xSemaphoreGive(i2cMutex);
            }
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // ----- EMOTE MODE BRANCH -----
        if (emote_mode_enabled && bootSequenceComplete) {
            joy_fwd = joy_side = joy_spin = 0.0f;

            float target_height = EMOTE_BODY_HEIGHT;
            float height_error  = target_height - current_body_height;
            float max_step      = EMOTE_HEIGHT_SPEED * DT_SEC;
            if (abs(height_error) > max_step) {
                current_body_height += (height_error > 0 ? 1.0f : -1.0f) * max_step;
            } else {
                current_body_height = target_height;
            }

            runEmoteTick();

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // ----- ACTIVE GAIT ENGINE -----
        float local_fwd  = is_fully_active ? joy_fwd  : 0.0f;
        float local_side = is_fully_active ? joy_side : 0.0f;
        float local_spin = is_fully_active ? joy_spin : 0.0f;

        // ----- STARTUP / MANUAL AUTO-CALIBRATION (bow/tilt dance, unchanged) -----
        if (!calDone() && !autoCalAttempted && !calRunnerIsRunning()) {
            if (footingIsStable()) {
                footingStableTimer += DT_SEC;
            } else {
                footingStableTimer = max(0.0f, footingStableTimer - DT_SEC * 0.5f);
            }
            if (footingStableTimer >= FOOTING_STABLE_S) {
                startupSettleTimer += DT_SEC;
                if (startupSettleTimer >= STARTUP_CAL_SETTLE_S) {
                    calRunnerStart();
                    autoCalAttempted = true;
                }
            }
        } else {
            footingStableTimer = 0.0f;
        }
        calRunnerStep(DT_SEC);
        bool cal_active = calRunnerIsRunning();
        if (cal_active) { local_fwd = local_side = local_spin = 0.0f; }

        // ---- Blend/mix the continuous gait command, then low-pass it ----
        GaitCmd raw = computeGaitCommand(local_fwd, local_side, local_spin);

        if (raw.timing_is_walk != timingIsWalk) {
            timingIsWalk   = raw.timing_is_walk;
            currentOffsets = timingIsWalk ? WALK_OFFSETS : TROT_OFFSETS;
            pending_blend_reset = true;
        }

        sm.freq         += (raw.freq - sm.freq) * PARAM_ALPHA;
        sm.duty          = min(max(sm.duty + (raw.duty - sm.duty) * PARAM_ALPHA, 0.3f), 0.9f);
        sm.amp_t         += (raw.amp_t - sm.amp_t) * PARAM_ALPHA;
        sm.amp_r         += (raw.amp_r - sm.amp_r) * PARAM_ALPHA;
        sm.sway          += (raw.sway - sm.sway) * PARAM_ALPHA;
        sm.straight_mix  += (raw.straight_mix - sm.straight_mix) * PARAM_ALPHA;
        sm.c = raw.c; sm.s = raw.s; sm.spin_dir = raw.spin_dir;

        // COG_OFFSET = Settings > Body Parameters > COG Offset (live-tunable fwd/back trim)
        urdf_x_filtered = (1.0f - URDF_X_ALPHA) * urdf_x_filtered + URDF_X_ALPHA * COG_OFFSET;

        // ----- LEVEL BLEND RAMP -----
        float target_level_blend = (balance_enabled && calDone() && !puppet_mode_enabled) ? 1.0f : 0.0f;
        level_blend += (target_level_blend - level_blend) * (DT_SEC / BALANCE_BLEND_RAMP_SEC);
        level_blend = constrain(level_blend, 0.0f, 1.0f);

        float work_freq = sm.freq, work_amp_t = sm.amp_t, work_amp_r = sm.amp_r;
        float liftHeight = walk_gait_enabled ? WALK_STEP_HEIGHT : STEP_HEIGHT;
        if (level_blend > 0.01f) {
            float scale = 1.0f - 0.2f * level_blend;
            work_freq *= scale; work_amp_t *= scale; work_amp_r *= scale;
        }

        bool active = is_fully_active && (sqrt(local_fwd*local_fwd + local_side*local_side) > 0.02f
                                          || fabs(local_spin) > 0.02f);
        if (active) {
            phase_accumulator = fmod(phase_accumulator + work_freq * DT_SEC, 1.0f);
        } else {
            phase_accumulator = 0.0f;
        }
        motion_blend += (((active) ? 1.0f : 0.0f) - motion_blend) * STOP_BLEND_SPEED;

        // ----- PUPPET MODE: slew + stale failsafe -----
        if (puppet_mode_enabled &&
            (millis() - puppet_last_rx_ms) > (unsigned long)(PUPPET_STALE_S * 1000.0f)) {
            puppet_mode_enabled = false;
            puppet_tgt_roll_deg = puppet_tgt_pitch_deg = puppet_tgt_yaw_deg = 0.0f;
            Serial.println("[Puppet] No data for 0.5s -- returning to level.");
        }
        if (!puppet_mode_enabled) {
            puppet_tgt_roll_deg = puppet_tgt_pitch_deg = puppet_tgt_yaw_deg = 0.0f;
        }
        puppet_cmd_roll_deg  = slewLimit(puppet_cmd_roll_deg,  puppet_tgt_roll_deg,  PUPPET_MAX_SLEW_DPS, DT_SEC);
        puppet_cmd_pitch_deg = slewLimit(puppet_cmd_pitch_deg, puppet_tgt_pitch_deg, PUPPET_MAX_SLEW_DPS, DT_SEC);
        puppet_cmd_yaw_deg   = slewLimit(puppet_cmd_yaw_deg,   puppet_tgt_yaw_deg,   PUPPET_MAX_SLEW_DPS, DT_SEC);

        if (puppet_mode_enabled) {
            user_selected_height = slewLimit(user_selected_height, PUPPET_BODY_HEIGHT,
                                             HEIGHT_SPEED, DT_SEC);
        }

        // ----- INACTIVITY TIMERS -----
        if (emote_mode_enabled || puppet_mode_enabled || in_calibration_mode
            || cal_active || balance_enabled || !is_fully_active) {
            last_activity_ms = millis();
        }
        unsigned long quiet_ms = millis() - last_activity_ms;

        if (AUTO_SLEEP_ENABLED && is_fully_active && target_standing_state
            && !balance_enabled) {
            unsigned long warn_at = (AUTO_SLEEP_MS > AUTO_SLEEP_WARN_MS)
                                    ? (AUTO_SLEEP_MS - AUTO_SLEEP_WARN_MS) : 0;
            if (quiet_ms >= AUTO_SLEEP_MS) {
                target_standing_state = false;
                auto_sleep_secs_left = -1;
                last_activity_ms = millis();
                Serial.println("[AutoSleep] Inactive -- sitting down.");
            } else if (quiet_ms >= warn_at) {
                auto_sleep_secs_left = (int)((AUTO_SLEEP_MS - quiet_ms + 999) / 1000);
            } else {
                auto_sleep_secs_left = -1;
            }
        } else {
            auto_sleep_secs_left = -1;
        }

        // ----- IDLE "ALIVE" MOTION -----
        bool idle_allowed = idle_motion_enabled && is_fully_active && !emote_mode_enabled
                            && !in_calibration_mode && !cal_active
                            && !puppet_mode_enabled
                            && quiet_ms >= IDLE_START_MS
                            && !balance_enabled && level_blend < 0.01f;
        float idle_activity = max(sqrt(local_fwd*local_fwd + local_side*local_side), fabs(local_spin));
        if (abs(manual_pitch_trim) > 1e-4f) idle_activity = 1.0f;
        idleUpdate(DT_SEC, idle_activity, idle_allowed);

        // ----- PITCH/ROLL FOR LEVELING -----
        static float pitchDegSmooth = 0.0f, rollDegSmooth = 0.0f;
        float pitchDegRawIn = degrees(body_pitch_filtered) - degrees(manual_pitch_trim);
        float rollDegRawIn  = degrees(body_roll_filtered);
        if (LEVEL_SMOOTH_TAU_S > 1e-4f) {
            float a = 1.0f - expf(-DT_SEC / LEVEL_SMOOTH_TAU_S);
            pitchDegSmooth += (pitchDegRawIn - pitchDegSmooth) * a;
            rollDegSmooth  += (rollDegRawIn  - rollDegSmooth)  * a;
        } else {
            pitchDegSmooth = pitchDegRawIn;
            rollDegSmooth  = rollDegRawIn;
        }
        float pitchDegRaw = pitchDegSmooth;
        float rollDegRaw  = rollDegSmooth;

        float pitchDegGated = (fabs(pitchDegRaw) >= LEVEL_DEADBAND_DEG) ? pitchDegRaw : 0.0f;
        float rollDegGated  = (fabs(rollDegRaw)  >= LEVEL_DEADBAND_DEG) ? rollDegRaw  : 0.0f;

        pitchDegGated = constrain(pitchDegGated, -LEVEL_MAX_TILT_DEG, LEVEL_MAX_TILT_DEG);
        rollDegGated  = constrain(rollDegGated,  -LEVEL_MAX_TILT_DEG, LEVEL_MAX_TILT_DEG);

        float pitch_error = radians(pitchDegGated);
        float roll_value  = radians(rollDegGated);

        // ----- HEIGHT -----
        float target_height = balance_enabled ? BALANCE_TARGET_HEIGHT : user_selected_height;
        float ramp_speed = exiting_emote_ramp ? EMOTE_HEIGHT_SPEED : HEIGHT_SPEED;
        float height_error  = target_height - current_body_height;
        float max_step_h    = ramp_speed * DT_SEC;
        if (abs(height_error) > max_step_h) {
            current_body_height += (height_error > 0 ? 1.0f : -1.0f) * max_step_h;
        } else {
            current_body_height = target_height;
            exiting_emote_ramp = false;
        }

        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            for (int idx = 0; idx < 4; idx++) {
                String legName = LEG_ORDER[idx];
                bool is_left  = legName.endsWith("L");
                bool is_right = legName.endsWith("R");
                float lx_root = LEG_ROOTS[idx][0];
                float ly_root = LEG_ROOTS[idx][1];

                // ---- Gait contribution (quintic/cycloid blend, ported from sim) ----
                float legOffset = currentOffsets[idx];
                float phase = fmod(phase_accumulator + legOffset, 1.0f);
                float pm_q, pm_c, lift_q, lift_c;
                legProfiles(phase, sm.duty, pm_q, pm_c, lift_q, lift_c);

                float mix_q = sm.straight_mix;
                float pm_t = mix_q * pm_q + (1.0f - mix_q) * pm_c;
                float lift_prof = mix_q * lift_q + (1.0f - mix_q) * lift_c;


                const float dir_sign = -1.0f;
                float lift_amt = liftHeight * motion_blend;
                float gait_dz = -1.0f * lift_amt * lift_prof;

                float gait_tx = dir_sign * work_amp_t * motion_blend * sm.c * pm_t;
                float gait_ty = dir_sign * work_amp_t * motion_blend * sm.s * pm_t + sm.sway * motion_blend;

                float spin_angle = dir_sign * work_amp_r * motion_blend * pm_c;
                gait_tx += -ly_root * spin_angle * sm.spin_dir;
                gait_ty +=  lx_root * spin_angle * sm.spin_dir;

                // ---- Trot<->walk timing-switch crossfade ----
                float gaitTarget[3] = { gait_tx, gait_ty, gait_dz };
                if (pending_blend_reset && have_last_leg_target) {
                    blend_offset[idx][0] = last_leg_target[idx][0] - gaitTarget[0];
                    blend_offset[idx][1] = last_leg_target[idx][1] - gaitTarget[1];
                    blend_offset[idx][2] = last_leg_target[idx][2] - gaitTarget[2];
                }
                for (int k = 0; k < 3; k++) {
                    blend_offset[idx][k] *= (1.0f - BLEND_ALPHA);
                    gaitTarget[k] += blend_offset[idx][k];
                    last_leg_target[idx][k] = gaitTarget[k];
                }
                gait_tx = gaitTarget[0]; gait_ty = gaitTarget[1]; gait_dz = gaitTarget[2];

                // ----- BODY LEVELING -----
                float tx_comp = 0.0f, ty_comp = 0.0f, delta_z = 0.0f;
                if (level_blend > 1e-4f && calDone()) {
                    computeBalanceCompensation(lx_root, ly_root, pitch_error, roll_value,
                                                LEVEL_GAIN, level_blend,
                                                tx_comp, ty_comp, delta_z);
                }

                // ----- PUPPET MODE OFFSETS -----
                if (fabsf(puppet_cmd_roll_deg) > 1e-3f || fabsf(puppet_cmd_pitch_deg) > 1e-3f
                    || fabsf(puppet_cmd_yaw_deg) > 1e-3f) {
                    float pdx = 0.0f, pdy = 0.0f, pdtz = 0.0f;
                    computePuppetFootOffsets(lx_root, ly_root,
                                              radians(puppet_cmd_roll_deg),
                                              radians(puppet_cmd_pitch_deg),
                                              radians(puppet_cmd_yaw_deg),
                                              pdx, pdy, pdtz);
                    tx_comp += pdx;
                    ty_comp += pdy;
                    delta_z += pdtz;
                }

                // ----- IDLE "ALIVE" OFFSETS -----
                if (idle_envelope > 1e-4f) {
                    float idle_tx = 0.0f, idle_ty = 0.0f, idle_dz = 0.0f;
                    computeBalanceCompensation(lx_root, ly_root,
                                                IDLE_ATTITUDE_SIGN * idle_pitch_rad,
                                                IDLE_ATTITUDE_SIGN * idle_roll_rad,
                                                1.0f, 1.0f,
                                                idle_tx, idle_ty, idle_dz);
                    tx_comp += idle_tx;
                    ty_comp += idle_ty;
                    delta_z += idle_dz;
                }

                // ----- MANUAL PITCH TRIM -----
                float manual_tx = 0.0f, manual_dz = 0.0f;
                computeManualPitchTilt(lx_root, ly_root, manual_pitch_trim, delta_z, tx_comp, manual_tx, manual_dz);

                // ----- BOW/TILT AUTO-CAL DANCE OFFSET -----
                float cal_dz = cal_active ? calRunnerLegDz(lx_root, ly_root) : 0.0f;

                float tx = gait_tx + tx_comp + manual_tx;
                float ty = gait_ty + ty_comp;
                float tz = current_body_height - idle_dz_m + delta_z + cal_dz + gait_dz + manual_dz;

                float current_h = 0.12 + (tz - 0.12) * smooth_progress;

                if (DEBUG_LEVEL_PRINT && idx == 0) {
                    static unsigned long lastLevelDebugMs = 0;
                    if (millis() - lastLevelDebugMs >= 500) {
                        lastLevelDebugMs = millis();
                        Serial.printf("[LevelDbg] pitch=%+.1fd roll=%+.1fd trim=%+.1fd err=%+.1fd "
                                      "dz=%+.4f gain=%.2f blend=%.2f timing=%s kill=%d\n",
                                      degrees(body_pitch_filtered), degrees(body_roll_filtered),
                                      degrees(manual_pitch_trim), degrees(pitch_error), delta_z,
                                      LEVEL_GAIN, level_blend, timingIsWalk ? "walk" : "trot",
                                      kill_switch_active);
                    }
                }

                float hip, thigh, knee;
                if (solve_leg_ik_3dof(tx, ty, current_h, urdf_x_filtered, hip, thigh, knee)) {
                    float actual_th = is_left ? -thigh : thigh;
                    float actual_kn = is_left ? -knee  : knee;
                    if (is_right) actual_th = -thigh;

                    float i_kn = is_left ? -KNEE_ASSEMBLY_OFFSET : KNEE_ASSEMBLY_OFFSET;

                    float f_h_real  = hip      * smooth_progress;
                    float f_th_real = actual_th * smooth_progress;
                    float f_kn_real = i_kn + (actual_kn - i_kn) * smooth_progress;

                    float h_phys_deg  = f_h_real  * 180.0 / PI;
                    float th_phys_deg = f_th_real * 180.0 / PI;
                    float kn_phys_deg = (f_kn_real - i_kn) * 180.0 / PI;

                    int hipHWID, thighHWID, kneeHWID;
                    legServoIds(legName, kneeHWID, thighHWID, hipHWID);
                    if (kneeHWID < 0) continue;

                    if (is_left) {
                        setServo(hipHWID,   90.0 - h_phys_deg);
                        setServo(thighHWID, 90.0 - th_phys_deg);
                        setServo(kneeHWID,  90.0 + kn_phys_deg);
                    } else {
                        setServo(hipHWID,   90.0 - h_phys_deg);
                        setServo(thighHWID, 90.0 + th_phys_deg);
                        setServo(kneeHWID,  90.0 + kn_phys_deg);
                    }
                }
            }
            xSemaphoreGive(i2cMutex);
        }

        pending_blend_reset = false;
        have_last_leg_target = true;

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

#endif // ROBOT_GAIT_MECHANISM_H
