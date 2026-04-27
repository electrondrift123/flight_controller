#include "tasks_config.h"

#include <STM32FreeRTOS.h>
#include <RF24.h>
// #include <RadioLib.h>

#include "QMC5883P.h"
#include "MPU6050.h"
#include "BMP280.h"
#include "Madgwick.h"

#include "pin_config.h"
#include "buzzer.h"
#include "shared_data.h"
#include "sync.h"

#include "main_rx.h"

#include "PID.h"
#include "LyGAPID.h"
#include "WDT.h"

#include "Butterworth2ndLPF.h"
#include "EMA.h"
#include "Mahony.h"
#include "calculus.h"

// DEFINE PRIORITY LEVELS (status: working)
#define PRIORITY_PID_FUSION   1 // 500 Hz -> try prio: 2 (radio task starve)
#define PRIORITY_WDT          1 // 1 Hz
#define PRIORITY_SENSOR_READ  1 // 1 kHz
#define PRIORITY_RADIO        1 // Interrupt driven
#define PRIORITY_BLINK        1 // 1 Hz

// Complementary filter for altitude estimation
float complementaryAltitude(float baroAlt,
                             float accN, float accE, float accD, // already NED
                             float dt){
  // --- Step 1: Gravity compensation (Down axis) ---
  float acc_z_corrected = (accD - 1.0f) * 9.81f;  // m/s², zero when hovering

  // --- Step 2: Integrate velocity ---
  static float vel_z = 0.0f;
  vel_z += acc_z_corrected * dt;

  // --- Step 3: Predict altitude ---
  static float alt_est = 0.0f;
  float alt_pred = alt_est + vel_z * dt;

  // --- Drift clamp / correction ---
  float err = baroAlt - alt_pred;
  if (fabsf(err) > 2.0f) {       // 2m threshold, tune
    vel_z *= 0.5f;               // dampen velocity instead of killing it
    alt_pred += err * 0.5f;      // pull prediction halfway toward baro
  }

  // --- Step 4: Complementary fuse with baro ---
  float alpha = 0.60f;   // tune
  alt_est = alpha * alt_pred + (1.0f - alpha) * baroAlt;

  return alt_est;
}

void buzz_it(uint16_t num, uint16_t delay_time = 500) {
  for (uint16_t i = 0; i < num; i++) {
    buzz_on(); // Buzzer ON
    vTaskDelay(pdMS_TO_TICKS(delay_time)); // Use FreeRTOS delay
    buzz_off(); // Buzzer OFF
    vTaskDelay(pdMS_TO_TICKS(delay_time));
  }
}

static inline float constrainFloat(float val, float minVal, float maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

// check sensor function
bool sensorsReady(){
  return BMP280_read(&bmpData) && MPU6050_read(&mpuData) && QMC5883P_read(&magData);
}

// === Timer callback (runs in timer task context!) ===
static void linkTimeoutCallback(TimerHandle_t xTimer){
  (void) xTimer;

  connection_ok = false;

  // Safest & fastest way for very short write
  taskENTER_CRITICAL();
  // inputList[0] = 0.0f; // we will not kill T for E-landing
  inputList[1] = 0.0f;
  inputList[2] = 0.0f;
  inputList[3] = 0.0f;
  if (inputList[4] == 0.0f) inputList[5] = 1.0f;   // set E-landing flag;   
  taskEXIT_CRITICAL();
}
void initLinkWatchdog(void){
  if (linkWatchdogTimer != NULL) {
    // already created → avoid double creation
    return;
  }

  linkWatchdogTimer = xTimerCreate(
    "LinkWD",
    LINK_TIMEOUT_MS / portTICK_PERIOD_MS, // timer period in ticks
    pdTRUE,                 // auto-reload
    NULL,
    linkTimeoutCallback
  );

  if (linkWatchdogTimer == NULL) {
      // Critical error - handle somehow (LED blink fast, Serial error, etc.)
      // For development you can leave it like this:
      // for(;;) { /* fail safe loop */ }
      connection_ok = false;
      taskENTER_CRITICAL();
      inputList[4] = 1.0f;  // stay killed
      taskEXIT_CRITICAL();
  }

  // We do NOT start it here — first valid packet will start/reset it
}

/////// TASKS /////// 
void watchdogTask(void* parameters) {
  vTaskDelay(pdMS_TO_TICKS(1000));
  IWDG_Init(); // watchdogtimer initialization for failsafes
  Serial.println("WDT init success!");

  const TickType_t interval = pdMS_TO_TICKS(200);
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    if (WDT_isSafe()) {
      IWDG->KR = 0xAAAA;  // Refresh watchdog
    } else { 
      // Don’t refresh → MCU resets
      if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(1)) == pdTRUE){
        Serial.println("Watchdog timed out");
        xSemaphoreGive(serialMutex);
      }
      buzz_on();
    }

    // debug print
    // if (xSemaphoreTake(serialMutex, portMAX_DELAY)){
    //   Serial.print("WDT_flag: "); Serial.println(WDT_isSafe());
    //   xSemaphoreGive(serialMutex);
    // }

    vTaskDelayUntil(&lastWakeTime, interval);
  }
}

void blinkTask(void *pvParameters) {
  TickType_t interval = pdMS_TO_TICKS(1000); // 500 ms interval
  for(;;){
    GPIOC->BSRR = (1 << (BUILTIN_LED_PIN)); // LED off
    vTaskDelay(interval);
    GPIOC->BSRR = (1 << (BUILTIN_LED_PIN + 16)); // LED on
    vTaskDelay(interval);
  }
}

void readSensorsTask(void* Parameters) {  // 1 kHz
  const TickType_t intervalTicks = pdMS_TO_TICKS(1);  // 1ms = 1 kHz
  TickType_t lastWakeTime = xTaskGetTickCount();

  float local_altitude; 
  float ax, ay, az, wx, wy, wz, mx, my, mz; // Madgwick

  // init for Accel LPF PT1
  emaInit(&axLPF, 1.0f, 3.0f, 1000.0f); // 15 Hz cutoff
  emaInit(&ayLPF, 1.0f, 3.0f, 1000.0f); // 15 Hz cutoff
  emaInit(&azLPF, 1.0f, 3.0f, 1000.0f); // 15 Hz cutoff

  emaInit(&raw_alt_LPF, 1.0f, 5.0f, 1000.0f); // PT1, fc = 1 Hz, fs = 1 kHz for raw altitude measurement smoothing
  emaInit(&vzLPF, 1.0f, 10.0f, 100.0f);

  float dt;

  static float altSmooth = 0.0f;

  // // old working tuning (not true az)
  // const float Q_pos = 0.0013f;   // process noise on position (was 0.013)
  // const float Q_vel = 0.03f;    // increase -> lower velocity bias
  // const float R_baro = 0.009f;    // better than 0.08 (faster and more accurate alt)

  // // new tuning (true az)
  // const float Q_pos = 0.0013f;   // process noise on position (was 0.013)
  // const float Q_vel = 0.005f;    // increase -> lower velocity bias
  // const float R_baro = 0.012f;    // better than 0.08 (faster and more accurate alt)

  // init_kalmanAltitude(&kalmanState, 0.0f, Q_pos, Q_vel, R_baro); // Q_pos, Q_vel, R_baro - tune these parameters based on your system's noise characteristics

  kalman_init(&kalmanState); // Initialize Kalman filter state

  Derivative_t altitude_derivative;
  init_derivative(&altitude_derivative);

  static int kalman_counter = 0; // run every 100 Hz
  const int kalman_interval = 10; // 10 ms = 100 Hz

  float fusedAlt = 0.0f;
  float velocity_z = 0.0f;
  float accel_bias = 0.0f;

  // compl
  EMA_t alt_comp_LPF;
  EMA_t vz_comp_LPF;
  emaInit(&alt_comp_LPF, 1.0f, 10.0f, 100.0f); // was fc = 2Hz
  emaInit(&vz_comp_LPF, 1.0f, 5.0f, 100.0f);
  float vz_az_comp = 0.0f;
  float vz_comp = 0.0f;
  float alt_prev = 0.0f;

  float az_true = 0.0f; // in g units
  float az_true_f = 0.0f;
  emaInit(&azTrueLPF, 1.0f, 10.0f, 1000.0f); // PT1 for vertical acceleration in world frame, fc = 2 Hz, fs = 1 kHz (for Kalman fusion)

  for (;;) {
    dt = intervalTicks * portTICK_PERIOD_MS / 1000.0f; // dt in seconds

    kalman_counter++;

    // read the BMP280 sensor
    if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(1)) == pdTRUE){
      // read MPU6050, BMP280, Magnetometer sensor: 
      if (sensorsReady()) {
        WDT_setSafe(true);

        // Successfully read MPU6050 data (acc_xyz is in g-units)
        ax = mpuData.ax;
        ay = mpuData.ay;
        az = mpuData.az;
        wx = mpuData.wx;
        wy = mpuData.wy;
        wz = mpuData.wz;
        mx = magData.mx;
        my = magData.my;
        mz = magData.mz;

        // Apply EMA filter to accelerometer data
        emaUpdate(&axLPF, ax);
        emaUpdate(&ayLPF, ay);
        emaUpdate(&azLPF, az);

        ax = axLPF.output; // Get filtered X
        ay = ayLPF.output; // Get filtered Y
        az = azLPF.output; // Get filtered Z

        local_altitude = bmpData.altitude; // Store altitude locally
       
      }else {
        if (xSemaphoreTake(serialMutex, portMAX_DELAY)){
          Serial.println("Error reading sensors");
        }
        // local_altitude = 0.0f; // Set to zero if read fails
        buzz_on();
        vTaskDelay(pdMS_TO_TICKS(100));
        buzz_off();
        WDT_setSafe(false);
      }
      xSemaphoreGive(wireMutex);
    }

    // update the Madgwick's data:
    if (xSemaphoreTake(madgwickMutex, pdMS_TO_TICKS(1)) == pdTRUE){
      MadgwickSensorList[0] = ax; // Accel X
      MadgwickSensorList[1] = ay; // Accel Y
      MadgwickSensorList[2] = az; // Accel Z
      MadgwickSensorList[3] = wx; // Gyro X
      MadgwickSensorList[4] = wy; // Gyro Y
      MadgwickSensorList[5] = wz; // Gyro Z
      MadgwickSensorList[6] = mx; // Mag X
      MadgwickSensorList[7] = my; // Mag Y
      MadgwickSensorList[8] = mz; // Mag Z

      az_true = getTrueAz(&madData, az);  // get vertical acceleration in g units

      xSemaphoreGive(madgwickMutex);
    }

    // --- call LP filter ---
    emaUpdate(&raw_alt_LPF, local_altitude); // Update raw altitude LPF
    local_altitude = raw_alt_LPF.output; // Get smoothed altitude for fusion

    emaUpdate(&azTrueLPF, az_true); // LPF for vertical acceleration in world frame

    if (kalman_counter >= kalman_interval) { // 100 Hz
      az_true_f = azTrueLPF.output; // Get smoothed vertical acceleration in world frame for fusion
      if (fabsf(az_true_f - 1.0f) < 0.10f) {  // Ignore small accelerations
        az_true_f = 1.0f;
      }
      az_true_f = (az_true_f - 1.0f) * 9.81f; 

      // fusedAlt = kalmanAltitudeUpdate(&kalmanState, local_altitude, az_world_filtered, dt * kalman_interval); // Kalman filter update for altitude estimation
      kalman_update(&kalmanState, local_altitude, az_true_f, dt * kalman_interval); // Kalman filter update for altitude estimation
      // kalman_update(&kalmanState, local_altitude, az_true_f, 0.010f); // Kalman filter update for altitude estimation
      fusedAlt = kalmanState.S[0]; // Get altitude estimate from Kalman filter
      velocity_z = kalmanState.S[1]; // Get vertical velocity estimate from Kalman filter
      accel_bias = kalmanState.S[2]; // see if converge to -0.30

      // also read velocity from Kalman state for future control
      // velocity_z = kalmanState.x[1]; // vertical velocity estimate from Kalman filter
      emaUpdate(&vzLPF, velocity_z);
      // velocity_z = vzLPF.output; // Get smoothed vertical velocity

      // if (fabsf(fusedAlt) < 0.15f) fusedAlt = 0.0f; // prevent small noise around zero altitude
      float vz_deadband = 0.15f; // was 20
      // if (fabs(az_true_f) < 0.1f && fabs(velocity_z) < 0.3f) vz_deadband = 0.05f;
      if (fabsf(velocity_z) < vz_deadband) velocity_z = 0.0f; // deadband to prevent jitter around zero velocity

      // leakage for vz
      const float vz_leak = 0.999f;
      velocity_z *= vz_leak;

      kalman_counter = 0; // reset counter
    }

    // Update shared data: BMP280 altitude
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(1)) == pdTRUE){
      telemetry[0] = fusedAlt; // Update altitude in telemetry
      telemetry[1] = velocity_z; // Update vertical velocity in telemetry
      xSemaphoreGive(telemetryMutex);
    }

    // // debug printf
    // if (xSemaphoreTake(serialMutex, portMAX_DELAY)){
    //   // Serial.print("Euler: ");
    //   // Serial.print(madData.roll); Serial.print(", ");
    //   // Serial.print(madData.pitch); Serial.print(", ");
    //   // Serial.println(madData.yaw);

    //   Serial.print("Vz: "); Serial.print(velocity_z); Serial.print(", ");
    //   Serial.print("alt: "); Serial.print(fusedAlt); Serial.print(", bias: ");
    //   Serial.println(accel_bias); 

    //   // Serial.print("az true f: "); Serial.println(az_true_f);

    //   // Serial.println(az_world); // 1g at ground, positive upward
    //   // Serial.print("raw alt: "); Serial.print(local_altitude); Serial.print(", az true: ");
    //   // Serial.println(((az_world - 1.0f) * 9.81));

    //   // Serial.print(mx); Serial.print(", "); Serial.print(my); Serial.print(", "); Serial.println(mz);
    //   xSemaphoreGive(serialMutex);
    // }

    vTaskDelayUntil(&lastWakeTime, intervalTicks); 
  }
}

void PIDtask(void* Parameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t interval = pdMS_TO_TICKS(2);  // 500 Hz
  const float dt = 0.002f;

  // ====================== CONFIG ======================
  const float BASE_THROTTLE = 1000.0f;      // Fixed base in mixer
  const float MAX_TB        = 545.0f;       // Maximum hover component for takeoff only (was 53% too low, 58% too high), [54, 55]
  const float HOVER_TB      = MAX_TB;       // Starting guess - tune this later

  const float KP_VZ = 140.0f; // was 160: [140, 180]
  const float KI_VZ = 0.5f; // was 0.5 good, [0.5, 2]
  const float VZ_OUTPUT_LIMIT = 200.0f; // was 250

  const float ARM_HOLD_TIME = 2.5f;

  const float MOTOR_MIN = 1050.0f;
  const float MOTOR_MAX = 2000.0f;

  bool KILL_MOTORS = false;
  bool E_LAND = false; // Emergency Landing flag if signal is loss

  // ====================== VARIABLES ======================
  float vz_cmd = 0.0f;                    // Raw stick [-80,100]
  float vz_cmdFiltered = 0.0f;            // in m/s
  float velocity_z = 0.0f;
  float altitude = 0.0f;

  float roll, pitch, yaw;
  float rollRate, pitchRate, yawRate;

  float rollInput, pitchInput, yawInput;

  float rollInputFiltered = 0.0f;
  float pitchInputFiltered = 0.0f;
  float yawInputFiltered = 0.0f;

  // EMA filters
  emaInit(&R_LPF, 1.0f, 15.0f, 500.0f);
  emaInit(&P_LPF, 1.0f, 15.0f, 500.0f);
  emaInit(&Y_LPF, 1.0f, 20.0f, 500.0f);
  emaInit(&T_LPF, 1.0f, 12.0f, 500.0f);

  // vertical controller mode
  typedef enum {
    VELOCITY_CONTROL,
    ALTITUDE_CONTROL
  } VerticalControlMode_t;

  VerticalControlMode_t vertical_mode = VELOCITY_CONTROL; // @ 100 Hz

  // State machine
  typedef enum {
    DISARMED,
    ARMED_IDLE,
    TAKEOFF,
    FLYING
  } FlightState_t;

  FlightState_t flightState = DISARMED;
  float armingTimer = 0.0f;
  float takeoffRamp = 0.0f;

  float vz_correction = 0.0f;
  float tb = 0.0f;                        // 0 → hover (this is your hover component)
  float Vb = 0.0; // 3s lipo: [11.4V, 12.6V]

  float motor_cmd[4] = {MOTOR_MIN, MOTOR_MIN, MOTOR_MIN, MOTOR_MIN};

  initVelocityControlZ(&vz_in, KP_VZ, KI_VZ, VZ_OUTPUT_LIMIT);

  float roll_rate_setpoint = 0.0f;
  float pitch_rate_setpoint = 0.0f;
  float yaw_rate_setpoint = 0.0f;

  float ax, ay, az, wx, wy, wz, mx, my, mz;

  int outer_loop_counter = 0;
  int print_counter = 0;

  vTaskDelay(pdMS_TO_TICKS(3000));

  for (;;) {
    // Sensor Read (Madgwick + Telemetry)
    if (xSemaphoreTake(madgwickMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      ax = MadgwickSensorList[0]; ay = MadgwickSensorList[1]; az = MadgwickSensorList[2];
      wx = MadgwickSensorList[3]; wy = MadgwickSensorList[4]; wz = MadgwickSensorList[5];
      mx = MadgwickSensorList[6]; my = MadgwickSensorList[7]; mz = MadgwickSensorList[8];
      xSemaphoreGive(madgwickMutex);
    }

    MadgwickFilterUpdate(&madData, wx, wy, wz, ax, ay, az, mx, my, mz, dt);
    MadgwickGetEuler(&madData);

    if (xSemaphoreTake(eulerAnglesMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      eulerAngles[0] = madData.roll;
      eulerAngles[1] = madData.pitch;
      eulerAngles[2] = madData.yaw;
      xSemaphoreGive(eulerAnglesMutex);
    }

    roll  = eulerAngles[0] * DEG_TO_RAD;
    pitch = eulerAngles[1] * DEG_TO_RAD;
    yaw   = eulerAngles[2] * DEG_TO_RAD;

    rollRate  = wx;
    pitchRate = wy;
    yawRate   = wz;

    // User Input
    if (xSemaphoreTake(nRF24Mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      vz_cmd     = inputList[0]; // [-0.8, 1.0] m/s
      yawInput   = inputList[1]; // [-180, 180] deg/s → rad/s
      pitchInput = inputList[2]; // [-20, 20] deg → rad
      rollInput  = inputList[3]; // [-20, 20] deg → rad

      KILL_MOTORS = (inputList[4] != 0.0f);
      bool pilot_E_LAND = (inputList[5] != 0.0f);

      // Pilot E_LAND switch OR low battery triggers emergency descent
      // But KILL_MOTORS overrides everything
      if (!KILL_MOTORS) {
        if (pilot_E_LAND || (Vb < 11.1f)) E_LAND = true;
        else E_LAND = false;
      }
      
      xSemaphoreGive(nRF24Mutex);
    }

    // Telemetry Update
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      altitude = telemetry[0];
      velocity_z = telemetry[1];
      Vb = telemetry[4] / 100.0f; // Convert to volts (3S LiPo)

      if (Vb > 12.6f) Vb = 12.6f;
      else if (Vb < 11.0f) Vb = 11.0f;

      xSemaphoreGive(telemetryMutex);
    }

    // E_LAND Behavior (execute in control loop, not in semaphore blocks)
    if (E_LAND && !KILL_MOTORS) {
      // Force descent unless pilot is actively climbing
      if (vz_cmd < 0.1f) {  // Not commanding significant climb
        vz_cmd = -0.3f;   // Safe descent rate, was 0.5
      }
      // If pilot gives positive vz_cmd, let them override
    }

    if (flightState == DISARMED || flightState == ARMED_IDLE) { 
      altitude = 0.0f;
      velocity_z = 0.0f;
      // Also reset Kalman alt & velocity state if possible (also the P matrix)
      // reset_kalmanAltitude(&kalmanState); // Reset Kalman filter state
      kalman_reset(&kalmanState);
    }

    if ((flightState == TAKEOFF || flightState == FLYING) && 
      (fabsf(roll) > MAX_SAFE_ANGLE_RAD || fabsf(pitch) > MAX_SAFE_ANGLE_RAD)) {
      KILL_MOTORS = true;
      flightState = DISARMED;
    }

    // ====================== ARMING (DJI Style) ======================
    bool sticksInArmPosition = (vz_cmd < -0.70f) && (fabsf(pitchInput) > 18.0f * DEG_TO_RAD);

    if (flightState == DISARMED) {
      if (sticksInArmPosition) {
        armingTimer += dt;
        if (armingTimer >= ARM_HOLD_TIME) {
          flightState = ARMED_IDLE;
          armingTimer = 0.0f;
          tb = 0.0f;
          buzz_it(1, 120);                    // One short beep on arm
          for (int i = 0; i < 4; i++) motor_cmd[i] = 1150.0f;
        }
      } else {
        armingTimer = 0.0f;
      }
    }

    // ====================== STATE MACHINE ======================
    switch (flightState) {
      case DISARMED:
        tb = 0.0f;
        vz_in.is_flying = 0.0f;
        break;

      case ARMED_IDLE:
        tb = 0.0f;
        vz_in.is_flying = 0.0f;
        if (vz_cmd > -0.70f) { 
          flightState = TAKEOFF;
          takeoffRamp = 0.0f;
        }
        break;

      case TAKEOFF:
        vz_in.is_flying = 1.0f;
        takeoffRamp += dt * 0.75f;           // ~1.3s ramp
        if (takeoffRamp > 1.0f) takeoffRamp = 1.0f;

        tb = takeoffRamp * MAX_TB;           // ramp 0 → 500

        if (takeoffRamp >= 1.0f && fabsf(velocity_z) < 0.5f) {
          flightState = FLYING;
        }
        break;

      case FLYING:
        vz_in.is_flying = 1.0f;
        // ====================== GROUND EFFECT COMPENSATION (E-LAND) ======================
        const float K_GE = 200.0f;  // Tune this (start 80-120)
        const float GE_THRESHOLD = 1.4f;
        const float R_PROP = 0.127f;  // 1045-inch prop radius
        float ge_comp = 0.0f;

        if (altitude < GE_THRESHOLD && E_LAND) {
          float z = altitude; // distance from ground in meters
          if (z < 0.3) z = 0.3; // prevent extreme compensation when very close to ground
          ge_comp = K_GE * (R_PROP / z) * (R_PROP / z);
        }

        static float Vb_hover_gain = 50.0f; // was 40
        tb = HOVER_TB + Vb_hover_gain * (12.6f - Vb) - ge_comp; // voltage sag compensation     
        break;
    }

    // ====================== FILTERING ======================
    emaUpdate(&R_LPF, rollInput);
    emaUpdate(&P_LPF, pitchInput);
    emaUpdate(&Y_LPF, yawInput);
    emaUpdate(&T_LPF, vz_cmd);

    rollInputFiltered  = constrainFloat(R_LPF.output, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
    pitchInputFiltered = constrainFloat(P_LPF.output, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
    yawInputFiltered   = constrainFloat(Y_LPF.output, -YAW_MAX, YAW_MAX);
    vz_cmdFiltered     = T_LPF.output;   // [-0.8,1.0] m/s

    // Attitude outer loop @ 100 Hz
    outer_loop_counter++;
    if (outer_loop_counter >= 5) {
      // Vertical Velocity Controller
      if ((flightState == TAKEOFF || flightState == FLYING) && vertical_mode == VELOCITY_CONTROL) {
        vz_correction = computeVelocityControlZ(&vz_in, vz_cmdFiltered, velocity_z, dt);
      }
      roll_rate_setpoint  = computeLyGAPID_out(&pidRoll,  rollInputFiltered,  roll,  0.01f);
      pitch_rate_setpoint = computeLyGAPID_out(&pidPitch, pitchInputFiltered, pitch, 0.01f);
      outer_loop_counter = 0;
    }

    yaw_rate_setpoint = yawInputFiltered;

    bool isLanded = (flightState == DISARMED || flightState == ARMED_IDLE);
    pidRoll.landed = pidPitch.landed = pidRollRate.landed = pidPitchRate.landed = isLanded ? 1.0f : 0.0f;

    float total_throttle = BASE_THROTTLE + tb + vz_correction;

    // Apply appropriate limits per state
    if (flightState == TAKEOFF || flightState == FLYING) {
      total_throttle = constrainFloat(total_throttle, 1450.0f, 1750.0f);
    }
    // DISARMED and ARMED_IDLE handle their own motor values directly

    // Motor Mixer
    if (KILL_MOTORS) {
      resetLyGAPID(&pidRoll); resetLyGAPID(&pidPitch);
      resetLyGAPID(&pidRollRate); resetLyGAPID(&pidPitchRate); resetLyGAPID(&pidYawRate);
      // reset_kalmanAltitude(&kalmanState);
      kalman_reset(&kalmanState);
      flightState = DISARMED;
      tb = 0.0f;
      vz_in.is_flying = 0.0f;
      pidRoll.landed = pidPitch.landed = pidRollRate.landed = pidPitchRate.landed = 1.0f;
      for (int i = 0; i < 4; i++) motor_cmd[i] = MOTOR_MIN;
    } else {
      float R_mix = constrainFloat(computeLyGAPID_in(&pidRollRate, roll_rate_setpoint, rollRate, dt), -U_MAX_ROLL_RATE, U_MAX_ROLL_RATE);
      float P_mix = constrainFloat(computeLyGAPID_in(&pidPitchRate, pitch_rate_setpoint, pitchRate, dt), -U_MAX_PITCH_RATE, U_MAX_PITCH_RATE);
      float Y_mix = constrainFloat(computeLyGAPID_yaw(&pidYawRate, yaw_rate_setpoint, yawRate, dt), -U_MAX_YAW_RATE, U_MAX_YAW_RATE);

      motor_cmd[0] = total_throttle + R_mix + P_mix - Y_mix;
      motor_cmd[1] = total_throttle - R_mix + P_mix + Y_mix;
      motor_cmd[2] = total_throttle + R_mix - P_mix + Y_mix;
      motor_cmd[3] = total_throttle - R_mix - P_mix - Y_mix;
    }

    for (int i = 0; i < 4; i++) {
      motor_cmd[i] = constrainFloat(motor_cmd[i], MOTOR_MIN, MOTOR_MAX);
    }

    // Motor Output
    TIM2->CCR1 = (uint16_t)motor_cmd[0];
    TIM2->CCR2 = (uint16_t)motor_cmd[1];
    TIM2->CCR3 = (uint16_t)motor_cmd[2];
    TIM2->CCR4 = (uint16_t)motor_cmd[3];

    // if (++print_counter >= 50) {
    //   print_counter = 0;
    // }

    vTaskDelayUntil(&lastWakeTime, interval);
  }
}

// // thesis PID task (CPU load)
// void PIDtask(void* Parameters) {
//   TickType_t lastWakeTime = xTaskGetTickCount();
//   const TickType_t interval = pdMS_TO_TICKS(2);  // 500 Hz
//   const float dt = 0.002f;

//   // ====================== CONFIG ======================
//   const float BASE_THROTTLE = 1000.0f;
//   const float MAX_TB        = 520.0f;
//   const float HOVER_TB      = 520.0f;

//   const float KP_VZ = 160.0f;
//   const float KI_VZ = 50.0f;
//   const float VZ_OUTPUT_LIMIT = 250.0f;

//   const float ARM_HOLD_TIME = 2.5f;

//   const float MOTOR_MIN = 1050.0f;
//   const float MOTOR_MAX = 2000.0f;

//   bool KILL_MOTORS = false;
//   bool E_LAND = false;

//   // ====================== VARIABLES ======================
//   float vz_cmd = 0.0f;
//   float vz_cmdFiltered = 0.0f;
//   float velocity_z = 0.0f;
//   float altitude = 0.0f;

//   float roll, pitch, yaw;
//   float rollRate, pitchRate, yawRate;

//   float rollInput, pitchInput, yawInput;

//   float rollInputFiltered = 0.0f;
//   float pitchInputFiltered = 0.0f;
//   float yawInputFiltered = 0.0f;

//   // EMA filters
//   emaInit(&R_LPF, 1.0f, 15.0f, 500.0f);
//   emaInit(&P_LPF, 1.0f, 15.0f, 500.0f);
//   emaInit(&Y_LPF, 1.0f, 20.0f, 500.0f);
//   emaInit(&T_LPF, 1.0f, 12.0f, 500.0f);

//   typedef enum {
//     VELOCITY_CONTROL,
//     ALTITUDE_CONTROL
//   } VerticalControlMode_t;

//   VerticalControlMode_t vertical_mode = VELOCITY_CONTROL;

//   typedef enum {
//     DISARMED,
//     ARMED_IDLE,
//     TAKEOFF,
//     FLYING
//   } FlightState_t;

//   FlightState_t flightState = DISARMED;
//   float armingTimer = 0.0f;
//   float takeoffRamp = 0.0f;

//   float vz_correction = 0.0f;
//   float tb = 0.0f;
//   float Vb = 0.0;

//   float motor_cmd[4] = {MOTOR_MIN, MOTOR_MIN, MOTOR_MIN, MOTOR_MIN};

//   initVelocityControlZ(&vz_in, KP_VZ, KI_VZ, VZ_OUTPUT_LIMIT);

//   float roll_rate_setpoint = 0.0f;
//   float pitch_rate_setpoint = 0.0f;
//   float yaw_rate_setpoint = 0.0f;

//   float ax, ay, az, wx, wy, wz, mx, my, mz;

//   int outer_loop_counter = 0;
//   int print_counter = 0;

//   // ====================== BENCHMARKING VARIABLES ======================
//   typedef struct {
//     uint64_t cycles_total;
//     uint32_t sample_count;
//     uint32_t min_cycles;
//     uint32_t max_cycles;
//     float last_mode;  // Track mode changes
//   } pid_benchmark_t;
  
//   static pid_benchmark_t bench = {
//     .cycles_total = 0,
//     .sample_count = 0,
//     .min_cycles = 0xFFFFFFFF,
//     .max_cycles = 0,
//     .last_mode = -1.0f  // Invalid initial value
//   };
  
//   // Initialize cycle counter
//   CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//   DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
//   DWT->CYCCNT = 0;
  
//   uint32_t cascade_start, cascade_end, cascade_cycles;
//   float current_mode;
  
//   vTaskDelay(pdMS_TO_TICKS(3000));

//   for (;;) {
//     // Sensor Read (Madgwick + Telemetry)
//     if (xSemaphoreTake(madgwickMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//       ax = MadgwickSensorList[0]; ay = MadgwickSensorList[1]; az = MadgwickSensorList[2];
//       wx = MadgwickSensorList[3]; wy = MadgwickSensorList[4]; wz = MadgwickSensorList[5];
//       mx = MadgwickSensorList[6]; my = MadgwickSensorList[7]; mz = MadgwickSensorList[8];
//       xSemaphoreGive(madgwickMutex);
//     }

//     MadgwickFilterUpdate(&madData, wx, wy, wz, ax, ay, az, mx, my, mz, dt);
//     MadgwickGetEuler(&madData);

//     if (xSemaphoreTake(eulerAnglesMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//       eulerAngles[0] = madData.roll;
//       eulerAngles[1] = madData.pitch;
//       eulerAngles[2] = madData.yaw;
//       xSemaphoreGive(eulerAnglesMutex);
//     }

//     roll  = eulerAngles[0] * DEG_TO_RAD;
//     pitch = eulerAngles[1] * DEG_TO_RAD;
//     yaw   = eulerAngles[2] * DEG_TO_RAD;

//     rollRate  = wx;
//     pitchRate = wy;
//     yawRate   = wz;

//     // User Input
//     if (xSemaphoreTake(nRF24Mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//       vz_cmd     = inputList[0];
//       yawInput   = inputList[1];
//       pitchInput = inputList[2];
//       rollInput  = inputList[3];

//       KILL_MOTORS = (inputList[4] != 0.0f);
//       bool pilot_E_LAND = (inputList[5] != 0.0f);

//       if (!KILL_MOTORS) {
//         if (pilot_E_LAND || (Vb < 11.1f)) E_LAND = true;
//         else E_LAND = false;
//       }
      
//       xSemaphoreGive(nRF24Mutex);
//     }

//     // Telemetry Update
//     if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//       altitude = telemetry[0];
//       velocity_z = telemetry[1];
//       Vb = telemetry[4] / 100.0f;
//       xSemaphoreGive(telemetryMutex);
//     }

//     if (E_LAND && !KILL_MOTORS) {
//       if (vz_cmd < 0.1f) {
//         vz_cmd = -0.5f;
//       }
//     }

//     if ((flightState == TAKEOFF || flightState == FLYING) && 
//       (fabsf(roll) > MAX_SAFE_ANGLE_RAD || fabsf(pitch) > MAX_SAFE_ANGLE_RAD)) {
//       KILL_MOTORS = true;
//       flightState = DISARMED;
//     }

//     // ====================== ARMING ======================
//     // bool sticksInArmPosition = (vz_cmd < -0.70f) && (fabsf(pitchInput) > 18.0f * DEG_TO_RAD);
//     bool sticksInArmPosition = true;

//     if (flightState == DISARMED) {
//       if (sticksInArmPosition) {
//         armingTimer += dt;
//         if (armingTimer >= ARM_HOLD_TIME) {
//           flightState = ARMED_IDLE;
//           armingTimer = 0.0f;
//           tb = 0.0f;
//           buzz_it(1, 120);
//           for (int i = 0; i < 4; i++) motor_cmd[i] = 1150.0f;
//         }
//       } else {
//         armingTimer = 0.0f;
//       }
//     }

//     // ====================== STATE MACHINE ======================
//     switch (flightState) {
//       case DISARMED:
//         tb = 0.0f;
//         vz_in.is_flying = 0.0f;
//         break;

//       case ARMED_IDLE:
//         tb = 0.0f;
//         vz_in.is_flying = 0.0f;
//         if (vz_cmd > -0.70f) { 
//           flightState = TAKEOFF;
//           takeoffRamp = 0.0f;
//         }
//         break;

//       case TAKEOFF:
//         vz_in.is_flying = 1.0f;
//         takeoffRamp += dt * 0.75f;
//         if (takeoffRamp > 1.0f) takeoffRamp = 1.0f;

//         tb = takeoffRamp * MAX_TB;

//         if (takeoffRamp >= 1.0f && fabsf(velocity_z) < 0.5f) {
//           flightState = FLYING;
//         }
//         break;

//       case FLYING:
//         vz_in.is_flying = 1.0f;
//         const float K_GE = 200.0f;
//         const float GE_THRESHOLD = 1.4f;
//         const float R_PROP = 0.127f;
//         float ge_comp = 0.0f;

//         if (altitude < GE_THRESHOLD && E_LAND) {
//           float z = altitude;
//           if (z < 0.3) z = 0.3;
//           ge_comp = K_GE * (R_PROP / z) * (R_PROP / z);
//         }

//         static float Vb_hover_gain = 40.0f;
//         tb = HOVER_TB + Vb_hover_gain * (12.6f - Vb) - ge_comp;
//         break;
//     }

//     // ====================== FILTERING ======================
//     emaUpdate(&R_LPF, rollInput);
//     emaUpdate(&P_LPF, pitchInput);
//     emaUpdate(&Y_LPF, yawInput);
//     emaUpdate(&T_LPF, vz_cmd);

//     rollInputFiltered  = constrainFloat(R_LPF.output, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
//     pitchInputFiltered = constrainFloat(P_LPF.output, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
//     yawInputFiltered   = constrainFloat(Y_LPF.output, -YAW_MAX, YAW_MAX);
//     vz_cmdFiltered     = T_LPF.output;

//     // Check for mode change
//     current_mode = pidRoll.mode;
//     if (current_mode != bench.last_mode) {
//       // Mode changed - reset benchmark data
//       bench.cycles_total = 0;
//       bench.sample_count = 0;
//       bench.min_cycles = 0xFFFFFFFF;
//       bench.max_cycles = 0;
//       bench.last_mode = current_mode;
//     }

//     // ====================== CASCADED P-PID MEASUREMENT START ======================
//     cascade_start = DWT->CYCCNT;
    
//     // Attitude outer loop @ 100 Hz
//     outer_loop_counter++;
//     if (outer_loop_counter >= 5) {
//       if ((flightState == TAKEOFF || flightState == FLYING) && vertical_mode == VELOCITY_CONTROL) {
//         vz_correction = computeVelocityControlZ(&vz_in, vz_cmdFiltered, velocity_z, dt);
//       }
//       roll_rate_setpoint  = computeLyGAPID_out(&pidRoll,  rollInputFiltered,  roll,  0.01f);
//       pitch_rate_setpoint = computeLyGAPID_out(&pidPitch, pitchInputFiltered, pitch, 0.01f);
//       outer_loop_counter = 0;
//     }

//     yaw_rate_setpoint = yawInputFiltered;

//     bool isLanded = (flightState == DISARMED || flightState == ARMED_IDLE);
//     pidRoll.landed = pidPitch.landed = pidRollRate.landed = pidPitchRate.landed = isLanded ? 1.0f : 0.0f;

//     // Inner loop - Rate PID controllers
//     float R_mix = constrainFloat(computeLyGAPID_in(&pidRollRate, roll_rate_setpoint, rollRate, dt), 
//                                  -U_MAX_ROLL_RATE, U_MAX_ROLL_RATE);
//     float P_mix = constrainFloat(computeLyGAPID_in(&pidPitchRate, pitch_rate_setpoint, pitchRate, dt), 
//                                  -U_MAX_PITCH_RATE, U_MAX_PITCH_RATE);
//     float Y_mix = constrainFloat(computeLyGAPID_yaw(&pidYawRate, yaw_rate_setpoint, yawRate, dt), 
//                                  -U_MAX_YAW_RATE, U_MAX_YAW_RATE);
    
//     cascade_end = DWT->CYCCNT;
//     cascade_cycles = cascade_end - cascade_start;
//     // ====================== CASCADED P-PID MEASUREMENT END ======================
    
//     // Store benchmark data only when flying
//     if (flightState == FLYING) {
//       bench.cycles_total += cascade_cycles;
//       bench.sample_count++;
      
//       if (cascade_cycles < bench.min_cycles) 
//         bench.min_cycles = cascade_cycles;
//       if (cascade_cycles > bench.max_cycles) 
//         bench.max_cycles = cascade_cycles;
//     }

//     float total_throttle = BASE_THROTTLE + tb + vz_correction;

//     if (flightState == TAKEOFF || flightState == FLYING) {
//       total_throttle = constrainFloat(total_throttle, 1450.0f, 1750.0f);
//     }

//     // Motor Mixer
//     if (KILL_MOTORS) {
//       resetLyGAPID(&pidRoll); resetLyGAPID(&pidPitch);
//       resetLyGAPID(&pidRollRate); resetLyGAPID(&pidPitchRate); resetLyGAPID(&pidYawRate);
//       kalman_reset(&kalmanState);
//       flightState = DISARMED;
//       tb = 0.0f;
//       vz_in.is_flying = 0.0f;
//       pidRoll.landed = pidPitch.landed = pidRollRate.landed = pidPitchRate.landed = 1.0f;
//       for (int i = 0; i < 4; i++) motor_cmd[i] = MOTOR_MIN;
//     } else {
//       motor_cmd[0] = total_throttle + R_mix + P_mix - Y_mix;
//       motor_cmd[1] = total_throttle - R_mix + P_mix + Y_mix;
//       motor_cmd[2] = total_throttle + R_mix - P_mix + Y_mix;
//       motor_cmd[3] = total_throttle - R_mix - P_mix - Y_mix;
//     }

//     for (int i = 0; i < 4; i++) {
//       motor_cmd[i] = constrainFloat(motor_cmd[i], MOTOR_MIN, MOTOR_MAX);
//     }

//     // Motor Output
//     TIM2->CCR1 = (uint16_t)motor_cmd[0];
//     TIM2->CCR2 = (uint16_t)motor_cmd[1];
//     TIM2->CCR3 = (uint16_t)motor_cmd[2];
//     TIM2->CCR4 = (uint16_t)motor_cmd[3];

//     // ====================== PERIODIC BENCHMARK REPORT ======================
//     print_counter++;
//     if (print_counter >= 2500) {  // Every 5 seconds at 500Hz
//       if (flightState == FLYING && bench.sample_count > 0) {
//         float cpu_freq = SystemCoreClock / 1000000.0f;
//         float avg_us = (bench.cycles_total / (float)bench.sample_count) / cpu_freq;
//         float min_us = bench.min_cycles / cpu_freq;
//         float max_us = bench.max_cycles / cpu_freq;
//         float cpu_load = (avg_us / 2000.0f) * 100.0f;
        
//         if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
//           Serial.println();
          
//           if (current_mode == 1.0f) {
//             Serial.println(F("STATIC PID BENCHMARK"));
//           } else {
//             Serial.println(F("ADAPTIVE PID BENCHMARK"));
//           }
          
//           Serial.println(F("├──────────────────────────────────────────────────────┤"));
          
//           Serial.print(F("Samples:  "));
//           Serial.print((unsigned int)bench.sample_count);
//           Serial.println(F("   "));
          
//           Serial.print(F("Average:  "));
//           Serial.print(avg_us, 2);
//           Serial.println(F(" us"));
          
//           Serial.print(F("Minimum:  "));
//           Serial.print(min_us, 2);
//           Serial.println(F(" us     "));
          
//           Serial.print(F("Maximum:  "));
//           Serial.print(max_us, 2);
//           Serial.println(F(" us   "));
          
//           Serial.print(F("CPU Load: "));
//           Serial.println(cpu_load, 2);

          
//           xSemaphoreGive(serialMutex);
//         }
//       }
//       print_counter = 0;
//     }

//     vTaskDelayUntil(&lastWakeTime, interval);
//   }
// }

///////////////////////////////////////////////////////////////
// // thesis PID task (telemetry): to be tested
// void PIDtask(void* Parameters) {
//   TickType_t lastWakeTime = xTaskGetTickCount();
//   const TickType_t interval = pdMS_TO_TICKS(2);  // 500 Hz
//   const float dt = 0.002f;

//   // ====================== CONFIG ======================
//   const float BASE_THROTTLE = 1000.0f;
//   const float MAX_TB        = 520.0f;
//   const float HOVER_TB      = 520.0f;

//   const float KP_VZ = 160.0f;
//   const float KI_VZ = 50.0f;
//   const float VZ_OUTPUT_LIMIT = 250.0f;

//   const float ARM_HOLD_TIME = 2.5f;

//   const float MOTOR_MIN = 1050.0f;
//   const float MOTOR_MAX = 2000.0f;

//   bool KILL_MOTORS = false;
//   bool E_LAND = false;

//   // ====================== VARIABLES ======================
//   float vz_cmd = 0.0f;
//   float vz_cmdFiltered = 0.0f;
//   float velocity_z = 0.0f;
//   float altitude = 0.0f;

//   float roll, pitch, yaw;
//   float rollRate, pitchRate, yawRate;

//   float rollInput, pitchInput, yawInput;

//   float rollInputFiltered = 0.0f;
//   float pitchInputFiltered = 0.0f;
//   float yawInputFiltered = 0.0f;

//   // EMA filters
//   emaInit(&R_LPF, 1.0f, 15.0f, 500.0f);
//   emaInit(&P_LPF, 1.0f, 15.0f, 500.0f);
//   emaInit(&Y_LPF, 1.0f, 20.0f, 500.0f);
//   emaInit(&T_LPF, 1.0f, 12.0f, 500.0f);

//   typedef enum {
//     VELOCITY_CONTROL,
//     ALTITUDE_CONTROL
//   } VerticalControlMode_t;

//   VerticalControlMode_t vertical_mode = VELOCITY_CONTROL;

//   typedef enum {
//     DISARMED,
//     ARMED_IDLE,
//     TAKEOFF,
//     FLYING
//   } FlightState_t;

//   FlightState_t flightState = DISARMED;
//   float armingTimer = 0.0f;
//   float takeoffRamp = 0.0f;

//   float vz_correction = 0.0f;
//   float tb = 0.0f;
//   float Vb = 0.0;

//   float motor_cmd[4] = {MOTOR_MIN, MOTOR_MIN, MOTOR_MIN, MOTOR_MIN};

//   initVelocityControlZ(&vz_in, KP_VZ, KI_VZ, VZ_OUTPUT_LIMIT);

//   float roll_rate_setpoint = 0.0f;
//   float pitch_rate_setpoint = 0.0f;
//   float yaw_rate_setpoint = 0.0f;

//   float ax, ay, az, wx, wy, wz, mx, my, mz;

//   int outer_loop_counter = 0;
//   int print_counter = 0;

//   // Local variables for telemetry
//   float R_mix, P_mix, Y_mix;

//   vTaskDelay(pdMS_TO_TICKS(3000));

//   for (;;) {
//     // Sensor Read (Madgwick + Telemetry)
//     if (xSemaphoreTake(madgwickMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//       ax = MadgwickSensorList[0]; ay = MadgwickSensorList[1]; az = MadgwickSensorList[2];
//       wx = MadgwickSensorList[3]; wy = MadgwickSensorList[4]; wz = MadgwickSensorList[5];
//       mx = MadgwickSensorList[6]; my = MadgwickSensorList[7]; mz = MadgwickSensorList[8];
//       xSemaphoreGive(madgwickMutex);
//     }

//     MadgwickFilterUpdate(&madData, wx, wy, wz, ax, ay, az, mx, my, mz, dt);
//     MadgwickGetEuler(&madData);

//     if (xSemaphoreTake(eulerAnglesMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//       eulerAngles[0] = madData.roll;
//       eulerAngles[1] = madData.pitch;
//       eulerAngles[2] = madData.yaw;
//       xSemaphoreGive(eulerAnglesMutex);
//     }

//     roll  = eulerAngles[0] * DEG_TO_RAD;
//     pitch = eulerAngles[1] * DEG_TO_RAD;
//     yaw   = eulerAngles[2] * DEG_TO_RAD;

//     rollRate  = wx;
//     pitchRate = wy;
//     yawRate   = wz;

//     // User Input
//     if (xSemaphoreTake(nRF24Mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//       vz_cmd     = inputList[0];
//       yawInput   = inputList[1];
//       pitchInput = inputList[2];
//       rollInput  = inputList[3];

//       KILL_MOTORS = (inputList[4] != 0.0f);
//       bool pilot_E_LAND = (inputList[5] != 0.0f);

//       if (!KILL_MOTORS) {
//         if (pilot_E_LAND || (Vb < 11.1f)) E_LAND = true;
//         else E_LAND = false;
//       }
      
//       xSemaphoreGive(nRF24Mutex);
//     }

//     // Telemetry Update
//     if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//       altitude = telemetry[0];
//       velocity_z = telemetry[1];
//       Vb = telemetry[4] / 100.0f;

//       xSemaphoreGive(telemetryMutex);
//     }

//     // E_LAND Behavior
//     if (E_LAND && !KILL_MOTORS) {
//       if (vz_cmd < 0.1f) {
//         vz_cmd = -0.5f;
//       }
//     }

//     if ((flightState == TAKEOFF || flightState == FLYING) && 
//       (fabsf(roll) > MAX_SAFE_ANGLE_RAD || fabsf(pitch) > MAX_SAFE_ANGLE_RAD)) {
//       KILL_MOTORS = true;
//       flightState = DISARMED;
//     }

//     // ====================== ARMING ======================
//     bool sticksInArmPosition = (vz_cmd < -0.70f) && (fabsf(pitchInput) > 18.0f * DEG_TO_RAD);

//     if (flightState == DISARMED) {
//       if (sticksInArmPosition) {
//         armingTimer += dt;
//         if (armingTimer >= ARM_HOLD_TIME) {
//           flightState = ARMED_IDLE;
//           armingTimer = 0.0f;
//           tb = 0.0f;
//           buzz_it(1, 120);
//           for (int i = 0; i < 4; i++) motor_cmd[i] = 1150.0f;
//         }
//       } else {
//         armingTimer = 0.0f;
//       }
//     }

//     // ====================== STATE MACHINE ======================
//     switch (flightState) {
//       case DISARMED:
//         tb = 0.0f;
//         vz_in.is_flying = 0.0f;
//         break;

//       case ARMED_IDLE:
//         tb = 0.0f;
//         vz_in.is_flying = 0.0f;
//         if (vz_cmd > -0.70f) { 
//           flightState = TAKEOFF;
//           takeoffRamp = 0.0f;
//         }
//         break;

//       case TAKEOFF:
//         vz_in.is_flying = 1.0f;
//         takeoffRamp += dt * 0.75f;
//         if (takeoffRamp > 1.0f) takeoffRamp = 1.0f;

//         tb = takeoffRamp * MAX_TB;

//         if (takeoffRamp >= 1.0f && fabsf(velocity_z) < 0.5f) {
//           flightState = FLYING;
//         }
//         break;

//       case FLYING:
//         vz_in.is_flying = 1.0f;
//         const float K_GE = 200.0f;
//         const float GE_THRESHOLD = 1.4f;
//         const float R_PROP = 0.127f;
//         float ge_comp = 0.0f;

//         if (altitude < GE_THRESHOLD && E_LAND) {
//           float z = altitude;
//           if (z < 0.3) z = 0.3;
//           ge_comp = K_GE * (R_PROP / z) * (R_PROP / z);
//         }

//         static float Vb_hover_gain = 40.0f;
//         tb = HOVER_TB + Vb_hover_gain * (12.6f - Vb) - ge_comp;
//         break;
//     }

//     // ====================== FILTERING ======================
//     emaUpdate(&R_LPF, rollInput);
//     emaUpdate(&P_LPF, pitchInput);
//     emaUpdate(&Y_LPF, yawInput);
//     emaUpdate(&T_LPF, vz_cmd);

//     rollInputFiltered  = constrainFloat(R_LPF.output, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
//     pitchInputFiltered = constrainFloat(P_LPF.output, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
//     yawInputFiltered   = constrainFloat(Y_LPF.output, -YAW_MAX, YAW_MAX);
//     vz_cmdFiltered     = T_LPF.output;

//     // Attitude outer loop @ 100 Hz
//     outer_loop_counter++;
//     if (outer_loop_counter >= 5) {
//       if ((flightState == TAKEOFF || flightState == FLYING) && vertical_mode == VELOCITY_CONTROL) {
//         vz_correction = computeVelocityControlZ(&vz_in, vz_cmdFiltered, velocity_z, dt);
//       }
//       roll_rate_setpoint  = computeLyGAPID_out(&pidRoll,  rollInputFiltered,  roll,  0.01f);
//       pitch_rate_setpoint = computeLyGAPID_out(&pidPitch, pitchInputFiltered, pitch, 0.01f);
//       outer_loop_counter = 0;
//     }

//     yaw_rate_setpoint = yawInputFiltered;

//     bool isLanded = (flightState == DISARMED || flightState == ARMED_IDLE);
//     pidRoll.landed = pidPitch.landed = pidRollRate.landed = pidPitchRate.landed = isLanded ? 1.0f : 0.0f;

//     // Inner loop - Rate PID controllers
//     R_mix = constrainFloat(computeLyGAPID_in(&pidRollRate, roll_rate_setpoint, rollRate, dt), 
//                            -U_MAX_ROLL_RATE, U_MAX_ROLL_RATE);
//     P_mix = constrainFloat(computeLyGAPID_in(&pidPitchRate, pitch_rate_setpoint, pitchRate, dt), 
//                            -U_MAX_PITCH_RATE, U_MAX_PITCH_RATE);
//     Y_mix = constrainFloat(computeLyGAPID_yaw(&pidYawRate, yaw_rate_setpoint, yawRate, dt), 
//                            -U_MAX_YAW_RATE, U_MAX_YAW_RATE);

//     float total_throttle = BASE_THROTTLE + tb + vz_correction;

//     if (flightState == TAKEOFF || flightState == FLYING) {
//       total_throttle = constrainFloat(total_throttle, 1450.0f, 1750.0f);
//     }

//     // ====================== THESIS TELEMETRY UPDATE ======================
//     if (xSemaphoreTake(thesisTelemetryMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//       // Control Effort
//       thesis_telemetry[0] = (int16_t)R_mix;
//       thesis_telemetry[1] = (int16_t)P_mix;
//       thesis_telemetry[2] = (int16_t)Y_mix;
      
//       // Angles and Rate
//       thesis_telemetry[3] = (int16_t)(roll * RAD_TO_DEG * 100);      // 0.01° resolution
//       thesis_telemetry[4] = (int16_t)(pitch * RAD_TO_DEG * 100);
//       thesis_telemetry[5] = (int16_t)(yawRate * RAD_TO_DEG * 10);    // 0.1 °/s resolution
      
//       // Yaw PI Gains
//       thesis_telemetry[6] = (int16_t)(pidYawRate.Kp * 100);
//       thesis_telemetry[7] = (int16_t)(pidYawRate.Ki * 100);
      
//       // Roll P-PID Gains
//       thesis_telemetry[8]  = (int16_t)(pidRoll.Kp * 100);
//       thesis_telemetry[9]  = (int16_t)(pidRollRate.Kp * 100);
//       thesis_telemetry[10] = (int16_t)(pidRollRate.Ki * 100);
//       thesis_telemetry[11] = (int16_t)(pidRollRate.Kd * 10000);
      
//       // Pitch P-PID Gains
//       thesis_telemetry[12] = (int16_t)(pidPitch.Kp * 100);
//       thesis_telemetry[13] = (int16_t)(pidPitchRate.Kp * 100);
//       thesis_telemetry[14] = (int16_t)(pidPitchRate.Ki * 100);
//       thesis_telemetry[15] = (int16_t)(pidPitchRate.Kd * 1000);
      
//       xSemaphoreGive(thesisTelemetryMutex);
//     }

//     // Motor Mixer
//     if (KILL_MOTORS) {
//       resetLyGAPID(&pidRoll); resetLyGAPID(&pidPitch);
//       resetLyGAPID(&pidRollRate); resetLyGAPID(&pidPitchRate); resetLyGAPID(&pidYawRate);
//       kalman_reset(&kalmanState);
//       flightState = DISARMED;
//       tb = 0.0f;
//       vz_in.is_flying = 0.0f;
//       pidRoll.landed = pidPitch.landed = pidRollRate.landed = pidPitchRate.landed = 1.0f;
//       for (int i = 0; i < 4; i++) motor_cmd[i] = MOTOR_MIN;
//     } else {
//       motor_cmd[0] = total_throttle + R_mix + P_mix - Y_mix;
//       motor_cmd[1] = total_throttle - R_mix + P_mix + Y_mix;
//       motor_cmd[2] = total_throttle + R_mix - P_mix + Y_mix;
//       motor_cmd[3] = total_throttle - R_mix - P_mix - Y_mix;
//     }

//     for (int i = 0; i < 4; i++) {
//       motor_cmd[i] = constrainFloat(motor_cmd[i], MOTOR_MIN, MOTOR_MAX);
//     }

//     // Motor Output
//     TIM2->CCR1 = (uint16_t)motor_cmd[0];
//     TIM2->CCR2 = (uint16_t)motor_cmd[1];
//     TIM2->CCR3 = (uint16_t)motor_cmd[2];
//     TIM2->CCR4 = (uint16_t)motor_cmd[3];

//     if (++print_counter >= 50) {
//       print_counter = 0;
//     }

//     vTaskDelayUntil(&lastWakeTime, interval);
//   }
// }

// // RXtask thesis (telemetry): testing....
// RXtask (thesis) - FIXED VERSION (minimal changes)
void RXtask(void* Parameters){
    int16_t local_telemetry[16] = {0};  // ACK payload
    int16_t rx_load[5] = {0, 0, 0, 0, 0};
    
    initLinkWatchdog();
    
    // CRITICAL: Pre-load initial ACK payload
    radio.writeAckPayload(PIPE_INDEX, local_telemetry, sizeof(local_telemetry));
    
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint8_t flags = radio.clearStatusFlags();
        
        if (flags & RF24_RX_DR) {
            while (radio.available()) {
                // Write ACK payload BEFORE read (already correct)
                radio.writeAckPayload(PIPE_INDEX, local_telemetry, sizeof(local_telemetry));
                
                // Small delay for PA/LNA module stability (optional, keep if needed)
                delayMicroseconds(100);
                
                // Read incoming data
                radio.read(rx_load, sizeof(rx_load));
                
                // ★★★ THE FIX - Add this one line ★★★
                radio.startListening();  // Force back to RX mode after ACK
                
                connection_ok = true;
                
                // Reset watchdog
                if (xTimerIsTimerActive(linkWatchdogTimer) == pdFALSE) {
                    xTimerStart(linkWatchdogTimer, 0);
                } else {
                    xTimerReset(linkWatchdogTimer, 0);
                }
                
                // Process incoming commands
                float Tcmd = (float)rx_load[0] / 100.0f;
                float Ycmd = ((float)rx_load[1]) * DEG_TO_RAD;
                float Pcmd = ((float)rx_load[2] / 100.0f) * DEG_TO_RAD * (-1.0f);
                float Rcmd = ((float)rx_load[3] / 100.0f) * DEG_TO_RAD;
                float kill = (rx_load[4] == 0) ? 0.0f : 1.0f;
                
                if (xSemaphoreTake(nRF24Mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                    inputList[0] = constrainFloat(Tcmd, THROTTLE_MIN, THROTTLE_MAX);
                    inputList[1] = constrainFloat(Ycmd, -YAW_MAX, YAW_MAX);
                    inputList[2] = constrainFloat(Pcmd, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
                    inputList[3] = constrainFloat(Rcmd, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
                    inputList[4] = kill;
                    xSemaphoreGive(nRF24Mutex);
                }
                
                // Prepare ACK payload for NEXT packet
                if (xSemaphoreTake(thesisTelemetryMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                    for (int i = 0; i < 16; i++) {
                        local_telemetry[i] = thesis_telemetry[i];
                    }
                    xSemaphoreGive(thesisTelemetryMutex);
                }
            }
        }
        
        // Handle MAX_RT if it occurs
        if (flags & RF24_TX_DF) {
            radio.flush_tx();
        }
    }
}
///////////////////////////////////////////////////////////////

// // dummy 1
// void RXtask(void* Parameters){
//   // reply: roll, pitch, yaw, alt, rad status, 2{P, kp, ki, kd}, {Kp, Ki, Kd}
//   // 32 bytes nRF24 max can handle, sent:  32 bytes, int16 = 2 bytes
//   int16_t local_telemetry[5] = {20, 20, 20, 20, 10}; 
//   int16_t rx_load[5] = {20, 20, 20, 20, 30};

//   int counter_print = 0;


//   // init failsafe radio link
//   initLinkWatchdog();

//   // Start killed
//   connection_ok = false;

//   for (;;) {
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//     // Read + clear status flags
//     uint8_t flags = radio.clearStatusFlags();

//     // Handle data ready
//     if (flags & RF24_RX_DR) {
//       while (radio.available()) {
//         // We are alive!
//         connection_ok = true;
//         Serial.println("help");

//         // Restart watchdog (this is the magic line)
//         // Critical: Start or restart watchdog
//         if (xTimerIsTimerActive(linkWatchdogTimer) == pdFALSE) {
//           xTimerStart(linkWatchdogTimer, 0);
//         } else {
//           xTimerReset(linkWatchdogTimer, 0);
//         }

       
//         // Write ACK payload BEFORE read
//         radio.writeAckPayload(PIPE_INDEX, local_telemetry, sizeof(local_telemetry));
        
//         // Read incoming data
//         radio.read(rx_load, sizeof(rx_load));
        
//         radio.startListening();

//         if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//           Serial.print("[nRF24 RX] Received: ");
//           for (int i = 0; i <= 3; i++) {
//             Serial.print(rx_load[i]); Serial.print(", ");
//           }
//           Serial.println(rx_load[4]);
//           xSemaphoreGive(serialMutex);
//         }

//         // // 2. update the data in inputList with mutex
//         // if (xSemaphoreTake(nRF24Mutex, pdMS_TO_TICKS(1)) == pdTRUE) {  
//         //   inputList[0] = Tcmd; // update throttle
//         //   inputList[1] = Ycmd; // update yaw
//         //   inputList[2] = Pcmd; // update pitch
//         //   inputList[3] = Rcmd; // update roll
//         //   inputList[4] = killcmd; // update kill flag in inputList
//         //   inputList[5] = e_landing; // Emergency landing flag

//         //   // temporary for analysis: roll only
//         //   inputList[6] = sigma; 
//         //   inputList[7] = gamma;
//         //   xSemaphoreGive(nRF24Mutex);
//         // }

//       }
//     }

//     // Optional: catch abnormal TX_ACK behavior
//     if (flags & 0x10) {
//       radio.flush_tx();  // clear stuck packet
//     }
//   }
// }


// // deepseek (working, but not very robust)
// void RXtask(void* Parameters){
//     int16_t local_telemetry[5] = {0, 0, 0, 0, 0};
//     int16_t rx_load[5] = {0, 0, 0, 0, 0};
//     bool first_packet = true;
    
//     initLinkWatchdog();
//     // radio.startListening();
    
//     for (;;) {
//         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//         uint8_t flags = radio.clearStatusFlags();
        
//         if (flags & RF24_RX_DR) {
//           while (radio.available()) {
//             radio.read(rx_load, sizeof(rx_load));
//             connection_ok = true;
            
//             // Reset watchdog
//             if (xTimerIsTimerActive(linkWatchdogTimer) == pdFALSE) xTimerStart(linkWatchdogTimer, 0);
//             else xTimerReset(linkWatchdogTimer, 0);
            
//             // Process incoming commands (scaling/clamping)
//             float Tcmd = (float)rx_load[0] / 100.0f; // Expected in the range [-80, 80] -> [-0.8, 0.8] m/s
//             float Ycmd = ((float)rx_load[1]) * DEG_TO_RAD; // [-90 deg/s, 90 deg/s]
//             float Pcmd = ((float)rx_load[2] / 100.0f) * DEG_TO_RAD * (-1.0f);
//             float Rcmd = ((float)rx_load[3] / 100.0f) * DEG_TO_RAD;
//             float kill = (rx_load[4] == 0) ? 0.0f : 1.0f;
//             if (xSemaphoreTake(nRF24Mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//               inputList[0] = constrainFloat(Tcmd, THROTTLE_MIN, THROTTLE_MAX);
//               inputList[1] = constrainFloat(Ycmd, -YAW_MAX, YAW_MAX);
//               inputList[2] = constrainFloat(Pcmd, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
//               inputList[3] = constrainFloat(Rcmd, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
//               inputList[4] = kill;
//               xSemaphoreGive(nRF24Mutex);
//             }
            
//             // Prepare ACK for NEXT packet
//             if (xSemaphoreTake(eulerAnglesMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//               local_telemetry[0] = (int16_t) eulerAngles[0];
//               local_telemetry[1] = (int16_t) eulerAngles[1];
//               local_telemetry[2] = (int16_t) eulerAngles[2];
//               xSemaphoreGive(eulerAnglesMutex);
//             }
//             if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//               local_telemetry[3] = (int16_t) (telemetry[0] * 100.0f);
//               local_telemetry[4] = (int16_t) telemetry[4];
//               xSemaphoreGive(telemetryMutex);
//             }
              
//             radio.writeAckPayload(PIPE_INDEX, local_telemetry, sizeof(local_telemetry));
//           }
//         }
        
//         // ONLY handle MAX_RT on RX side (rare, but possible)
//         if (flags & RF24_TX_DF) {
//             radio.flush_tx(); // Clear stuck ACK payload
//         }
//     }
// }

//// TODO: find ADC pin for this task (12 bit res: [12.6V: 2544, 11.4V: ?])
void batteryMonitorTask(void* Parameters){
  TickType_t interval = pdMS_TO_TICKS(20);
  TickType_t lastWakeTime = xTaskGetTickCount();

  float batteryVoltage; // Variable to hold battery voltage

  int counter = 0;
  float Vb_avg = 0.0f;

  emaInit(&VbLPF, 2.0f, 10.0f, 50.0f); // PT2, fc = 5 Hz, fs = 50 Hz (dt=0.02s) for battery voltage smoothing

  for (;;){
    // Read battery voltage
    counter++;
    uint16_t adc_value = readVbat();
    emaUpdate(&VbLPF, adc_value);
    adc_value = VbLPF.output; // Get filtered ADC value

    Vb_avg += (float)adc_value / 10.0f; // accumulate for averaging    

    if (counter == 10){
    batteryVoltage = Vb_avg * (12.6f / 2430.0f); // Convert ADC value to voltage (assuming 12.6V max and 12-bit ADC)

      // Update shared data
      if (xSemaphoreTake(telemetryMutex, portMAX_DELAY)) {
        telemetry[4] = (int16_t)(100.0f * batteryVoltage); // Update battery voltage in telemetry (scaled by 100)
        xSemaphoreGive(telemetryMutex);
      }

      if (batteryVoltage < 11.0f){
        buzz_on();
        vTaskDelay(pdMS_TO_TICKS(10));
        buzz_off();
      }

      // Serial.print("/Batt: "); Serial.print(batteryVoltage); Serial.println(" V");
      Vb_avg = 0.0f; // reset the voltage
      counter = 0; // reset the counter
    }
    
    vTaskDelayUntil(&lastWakeTime, interval); // Delay until the next cycle
  }
}

////////// freeRTOS INIT
BaseType_t result;
void freeRTOS_tasks_init(void){
  result = xTaskCreate(
    watchdogTask,
    "Watchdog Task",
    64,
    NULL,
    PRIORITY_WDT,
    NULL
  );
  if (result != pdPASS) {
    Serial.println("Failed to create the WDT task!");
    while(1);
  }

  result = xTaskCreate(
    blinkTask,           // Task function
    "blinkTask",         // Name of the task
    64,                // Stack size in words
    NULL,                // Task input parameter
    PRIORITY_BLINK,                   // Priority
    NULL                 // Task handle
  );
  if (result != pdPASS) {
    // Handle task creation failure
    Serial.println("Failed to create blinkTask");
    while (1); // Infinite loop to indicate failure
  }

  result = xTaskCreate(
    readSensorsTask,
    "read Sensors Task",
    128, // Stack size in words
    NULL,
    PRIORITY_SENSOR_READ,
    NULL
  );
  if (result != pdPASS) {
    // Handle task creation failure
    Serial.println("Failed to create readSensorsTask");
    while (1); // Infinite loop to indicate failure
  }

  result = xTaskCreate(
    PIDtask,
    "PID and Fusion task",
    256,
    NULL,
    PRIORITY_PID_FUSION,
    NULL
  );
  if (result != pdPASS) {
    // Handle task creation failure
    Serial.println("Failed to create PIDtask");
    while (1); // Infinite loop to indicate failure
  }

  result = xTaskCreate(
    RXtask,
    "nRF24 RX task",
    256,
    NULL,
    PRIORITY_RADIO,
    &radioTaskHandle
  );
  if (result != pdPASS) {
    // Handle task creation failure
    Serial.println("Failed to create RXtask");
    while (1); // Infinite loop to indicate failure
  }

  result = xTaskCreate(
    batteryMonitorTask,
    "Battery Monitor Task",
    256,
    NULL,
    1,
    NULL
  );
  if (result != pdPASS) {
    // Handle task creation failure
    Serial.println("Failed to create batteryMonitorTask");
    while (1); // Infinite loop to indicate failure
  }
}



// Priority tasks are all set to 1 (working) - i will try to change for the better: status: trying...

//// TODO: 
// - [y] test rod 
// - [y] fix the radio connectivity issue
// - [y] real test flight (assisted) 
// - [x] fix vertical control performance (estimation and controller tuning)
// - [x] real test flight (tethered) 
// - [x] free flight
// - [x] test flight with payload (varying/sloshing)

// - [x] cpu load data extraction

// - [x] data extraction flight test (static PID and LyGAPID: 4 runs: with & w/out sloshing)