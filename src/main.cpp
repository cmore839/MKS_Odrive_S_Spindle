#include "globals.h"

SPIClass spi3(SPI3_MOSO, SPI3_MISO, SPI3_SCL);
LowsideCurrentSense CS1 = LowsideCurrentSense(SHUNT_RESISTOR, CSA_GAIN, _NC, PIN_CUR_B, PIN_CUR_C, PIN_VBUS, VBUS_DIVIDER_RATIO, PIN_TEMP_M1);
BLDCMotor M1 = BLDCMotor(2, phase_resistance, kv_rating, q_phase_inductance);
BLDCDriver6PWM DR1 = BLDCDriver6PWM(M0_INH_A,M0_INL_A, M0_INH_B,M0_INL_B, M0_INH_C,M0_INL_C);
STM32HWEncoder E1 = STM32HWEncoder(1110, M0_ENC_A, M0_ENC_B, _NC);
//BLDCMotor M1 = BLDCMotor(5, 0.2816, 145, 0.00037);
//STM32HWEncoder E1 = STM32HWEncoder(16384, M0_ENC_A, M0_ENC_B, _NC);
//StepDirListener SD1 = StepDirListener(PA2, PA3, 0.006135f);//2*PI/1024=); // For 4096 steps/rev encoder
//void onStep() { SD1.handle(); } 


void setupSerial() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000); // Wait for Serial
  SimpleFOCDebug::enable(&Serial);
  Serial.println(F("Booting..."));
}

/**
 * @brief Configures and initializes the DRV8301 gate driver.
 * @param verbose If true, print all config and status messages.
 */
void setupDRV8301(bool verbose) {
    if (verbose) Serial.println(F("Setting up DRV8301..."));
    
    // Set pin modes for EN_GATE and nCS
    pinMode(EN_GATE, OUTPUT);
    pinMode(SPI3_CS, OUTPUT);
    
    digitalWrite(SPI3_CS, HIGH); // nCS idle high
    delay(1);
    
    // Enable the DRV8301
    digitalWrite(EN_GATE, HIGH);
    delay(10); // Wait for DRV8301 to power up

    // Start SPI3
    spi3.begin();
    
    // Initialize our global driver object
    drv.init(&spi3, SPI3_CS);
    drv.setVerbose(verbose); // Control verbosity

    // --- Read initial state ---
    if (verbose) drv.printStatus();
    
    // --- Configure DRV8301 ---
    if (verbose) Serial.println(F("Configuring DRV8301..."));
    
    bool config_ok = true;
    // Set 6-PWM mode (required by BLDCDriver6PWM)
    config_ok &= drv.setPWMMode(DRV8301_PwmMode_Six_Inputs);
    // Set Gain to 80V/V to match 'CSA_GAIN = 80' in globals.cpp
    config_ok &= drv.setGain(DRV8301_ShuntAmpGain_80VpV);
    // Set OC Mode to 'Report Only' - let SimpleFOC handle the fault.
    config_ok &= drv.setOcMode(DRV8301_OcMode_ReportOnly);
    // Set Gate Current to max
    config_ok &= drv.setGateCurrent(DRV8301_PeakCurrent_1p70_A);

    if (!config_ok) {
        Serial.println(F("!!! DRV8301 CONFIGURATION FAILED! HALTING !!!"));
        while(1); // Halt on config failure
    }

    // --- Read back and print final config ---
    if (verbose) {
        drv.printConfig();
        Serial.println(F("Final status check..."));
        drv.printStatus();
    }
}

void setupMotorParameters() {
  DR1.pwm_frequency = 25000;
  DR1.voltage_power_supply = 24.0;
  DR1.voltage_limit = 24.0;
//   DR1.voltage_power_supply = 56.0;
//   DR1.voltage_limit = 56.0;
  M1.motion_downsample = 10; // run the control loop at each foc loop
  M1.voltage_limit = 24.0;   // [V]
  //M1.voltage_limit = 56.0;   // [V]
  M1.current_limit = peak_current_limit; // FOC hard limit is the absolute PEAK
  M1.voltage_sensor_align = 3.0;
  M1.acceleration_limit = 500.0; //velocity mode only
  M1.velocity_limit = 50; // [rad/s]
  M1.torque_controller = TorqueControlType::foc_current;
  M1.controller = MotionControlType::angle;
  M1.foc_modulation = FOCModulationType::SpaceVectorPWM;
}

void setupMotorPIDs() {
  // velocity PID controller parameters
  M1.PID_velocity.P = 0.05;
  M1.PID_velocity.I = 0.0;
  //M1.PID_velocity.I = 1.0;
  M1.PID_velocity.D = 0;
  M1.PID_velocity.output_ramp = 0;
  M1.LPF_velocity.Tf = 0;
   
  // angle PID controller 
  M1.P_angle.P = 350.0;
  //M1.P_angle.P = 20.0;
  M1.P_angle.I = 0;
  M1.P_angle.D = 0;
  M1.P_angle.output_ramp = 0;
  M1.LPF_angle.Tf = 0;

  // Current PIDs
  M1.PID_current_q.P = q_phase_inductance*current_bandwidth*_2PI;
  M1.PID_current_q.I= M1.PID_current_q.P*phase_resistance/q_phase_inductance;
  M1.PID_current_d.P= d_phase_inductance*current_bandwidth*_2PI;
  M1.PID_current_d.I = M1.PID_current_d.P*phase_resistance/d_phase_inductance;
  M1.LPF_current_q.Tf = 1/(5.0*current_bandwidth); 
  M1.LPF_current_d.Tf = 1/(5.0*current_bandwidth);
}

void linkMotorComponents() {
  DR1.init();
  M1.linkDriver(&DR1);
  E1.init();
  M1.linkSensor(&E1);
  CS1.linkDriver(&DR1);
  CS1.linkMotorAndLimits(&M1, &continuous_current_limit); // <-- Pass the continuous limit global  
  CS1.init();
  CS1.setTemperatureLimits(low_temp_limit, high_temp_limit, motor_cutoff_temp);
  CS1.skip_align = true; // Set this before M1.initFOC()
  M1.linkCurrentSense(&CS1);
  
  M1.init();     // This will init driver and sensor
  M1.initFOC();  // This will init current sense
  
  // --- NEW SECTION: Read VBUS and update params ---
  Serial.println(F("Reading VBUS and setting limits..."));
  delay(10); // Give ADC a moment to get a stable reading
  
  float measured_vbus = CS1.getVbusVoltage();
  
  // Safety check
  if (measured_vbus < 3.0f || measured_vbus > 50.0f) { 
      measured_vbus = DR1.voltage_power_supply; // Stick with the 24V default
      Serial.println(F("WARN: VBUS read failed, using default 24V!"));
  }
  
  // Calculate new limits
  float voltage_limit = measured_vbus * 0.95f;
  float brake_target = measured_vbus + 0.5f;
  
  // Set the new values
  DR1.voltage_power_supply = measured_vbus;
  DR1.voltage_limit = voltage_limit;
  M1.voltage_limit = voltage_limit;
  M1.PID_current_q.limit = voltage_limit;
  
  // Initialize brake resistor HERE, now that we have the real voltage
  CS1.initBrakeResistorPWM(PIN_BRAKE_RESISTOR, brake_target, BRAKE_P_GAIN, BRAKE_I_GAIN);
  
  Serial.print(F("  VBUS: ")); Serial.print(measured_vbus); Serial.println(F(" V"));
  Serial.print(F("  Volt Limit: ")); Serial.print(voltage_limit); Serial.println(F(" V (95%)"));
  Serial.print(F("  Brake Target: ")); Serial.print(brake_target); Serial.println(F(" V"));
  // --- End of new section ---
}

/**
 * @brief Read all external sensors (Vbus, Temp, etc.)
 */
void updateExternalSensors() {
    current1 = CS1.getPhaseCurrents();
    VBUS_S = CS1.getVbusVoltage();
    CS1.updateBrakeResistor();
    CS1.checkTemperature();
    Temp_M1 = CS1.getTemperature();
}

/**
 * @brief Run all safety checks (Current, Following Error, Temp)
 */
void checkSafetyPack() {
    // 1. Current Limit Check
    float current_magnitude = _sqrt(M1.current.q * M1.current.q + M1.current.d * M1.current.d);

    if (current_magnitude > continuous_current_limit) {
        if (peak_current_start_time_ms == 0) {
            peak_current_start_time_ms = millis();
        } else if (millis() - peak_current_start_time_ms > peak_current_timeout_ms) {
            M1.disable();
            drive_disabled = true;
            Serial.println("ERROR: Sustained current over continuous limit! Drive disabled.");
        }
    } else {
        peak_current_start_time_ms = 0;
    }

    // 2. Following Error Check
    float following_error = 0;
    if (M1.controller == MotionControlType::angle) {
        following_error = target - M1.shaft_angle;
    } else if (M1.controller == MotionControlType::velocity) {
        following_error = target - M1.shaft_velocity;
    }

    if (M1.controller != MotionControlType::torque) {
        if (abs(following_error) > max_following_error) {
                if (following_error_start_time_ms == 0) {
                following_error_start_time_ms = millis();
                } else if (millis() - following_error_start_time_ms > max_following_error_timeout_ms) {
                M1.disable();
                drive_disabled = true;
                Serial.println("ERROR: Sustained following error! Drive disabled.");
                }
        } else {
            following_error_start_time_ms = 0;
        }
    }

    // 3. Temperature Check
    if (Temp_M1 > max_temp) {
        M1.disable();
        drive_disabled = true;
        Serial.println("ERROR: FET Over-temperature! Drive disabled.");
    }
}

/**
 * @brief Resets the drive from a fault condition.
 */
void resetDriveFault() {
    Serial.println(F("Resetting drive fault..."));
    drive_disabled = false;
    following_error_start_time_ms = 0;
    peak_current_start_time_ms = 0; // Also reset this one
    M1.enable();
    digitalWrite(EN_GATE, HIGH); // Re-enable DRV
    delay(10); // Give hardware a moment
}

/**
 * @brief Runs a stall-based end-to-end calibration. BLOCKING.
 * This now uses ANGLE mode with a ramping target to trigger on FOLLOWING ERROR.
 */
void calibrateAxis() {
    Serial.println(F("--- Starting Axis Calibration (Angle Mode) ---"));
    unsigned long last_run_us = micros();
    float dt_s = 0.0f;

    // Store original settings
    MotionControlType orig_controller = M1.controller;
    float orig_target = target;
    float orig_vel_limit = M1.velocity_limit;
    float orig_current_limit = peak_current_limit;
    float orig_cont_current_limit = continuous_current_limit; // <-- Store continuous limit
    float orig_following_error = max_following_error;

    // Set temporary "stall detection" limits
    Serial.println(F("Setting stall detection limits (High Current, Low Follow Error)..."));
    peak_current_limit = calibration_current_limit;
    M1.current_limit = calibration_current_limit; 
    continuous_current_limit = calibration_current_limit; // <-- Set continuous limit high
    max_following_error = calibration_following_error;  // <-- Set follow error low
    M1.controller = MotionControlType::angle; // <-- Use ANGLE mode
    M1.velocity_limit = calibration_velocity; 
    
    // --- Move to Negative End ---
    Serial.println(F("Moving to negative end stop..."));
    target = M1.shaft_angle; // Start ramp from current position
    resetDriveFault();
    last_run_us = micros();
    
    while (!drive_disabled) {
        dt_s = (micros() - last_run_us) * 1e-6f;
        if(dt_s < 0.00005f) continue; // 20kHz loop
        last_run_us = micros();

        target -= calibration_velocity * dt_s; // Ramp target negative
        M1.loopFOC();
        M1.move(target);
        checkSafetyPack(); 
    }
    float limit_1_angle = M1.shaft_angle; // Record actual stalled position
    Serial.print(F("Found negative end stop at [rad]: ")); Serial.println(limit_1_angle);

    // --- Move to Positive End ---
    Serial.println(F("Moving to positive end stop..."));
    target = M1.shaft_angle; // Start ramp from current position
    resetDriveFault(); // Reset fault from first stall
    last_run_us = micros();

    while (!drive_disabled) {
        dt_s = (micros() - last_run_us) * 1e-6f;
        if(dt_s < 0.00005f) continue; // 20kHz loop
        last_run_us = micros();

        target += calibration_velocity * dt_s; // Ramp target positive
        M1.loopFOC();
        M1.move(target);
        checkSafetyPack();
    }
    float limit_2_angle = M1.shaft_angle; // Record actual stalled position
    Serial.print(F("Found positive end stop at [rad]: ")); Serial.println(limit_2_angle);

    // Calculate stroke
    float total_stroke = abs(limit_2_angle - limit_1_angle);
    Serial.println(F("----------------------------------"));
    Serial.print(F(">>> DEBUG: TOTAL STROKE [rad]: ")); Serial.println(total_stroke);
    Serial.println(F(">>> Copy this value to 'known_stroke_distance' in globals.cpp"));
    Serial.println(F("----------------------------------"));


    // --- Home the axis ---
    Serial.println(F("Homing to negative end stop..."));
    resetDriveFault(); // Reset fault from second stall
    
    // Set "homing move" limits (High Follow Error)
    max_following_error = homing_following_error; 
    
    target = limit_1_angle; // Go to the recorded stall point
    long homing_start_ms = millis();
    
    while (abs(M1.shaft_angle - target) > 0.05f) { 
        if (millis() - homing_start_ms > 5000) {
            Serial.println(F("Homing failed! Timeout."));
            break;
        }
        M1.loopFOC();
        M1.move(target);
        checkSafetyPack(); 
        if(drive_disabled) {
           Serial.println(F("Homing failed! Drive disabled during homing."));
           break;
        }
    }
    
    if (!drive_disabled) {
        Serial.println(F("Homed. Setting new zero."));
        M1.sensor->update(); 
        M1.sensor_offset = M1.sensor_direction * M1.sensor->getAngle();
        
        soft_limit_negative = 0.0f + calibration_stroke_buffer;
        soft_limit_positive = total_stroke - calibration_stroke_buffer;

        if (soft_limit_positive <= soft_limit_negative) {
            Serial.println(F("WARN: Stroke buffer is too large! Disabling soft limits."));
            soft_limits_enabled = false;
        } else {
            soft_limits_enabled = true;
        }

        Serial.print(F("Calibration complete. New limits: "));
        Serial.print(soft_limit_negative); Serial.print(F(" to ")); Serial.println(soft_limit_positive);
    }

    // --- Restore original settings ---
    Serial.println(F("Restoring original limits..."));
    M1.controller = orig_controller;
    target = 0; // Set target to 0 (new home)
    M1.move(0); 
    M1.velocity_limit = orig_vel_limit;
    peak_current_limit = orig_current_limit;
    M1.current_limit = orig_current_limit; 
    continuous_current_limit = orig_cont_current_limit; // <-- Restore continuous limit
    max_following_error = orig_following_error;
    resetDriveFault(); // Final reset
}

/**
 * @brief Runs a stall-based homing sequence to the negative end. BLOCKING.
 * This now uses ANGLE mode with a ramping target to trigger on FOLLOWING ERROR.
 */
void homeAxis() {
    Serial.println(F("--- Starting Homing Sequence (Angle Mode) ---"));
    unsigned long last_run_us = micros();
    float dt_s = 0.0f;

    // Store original settings
    MotionControlType orig_controller = M1.controller;
    float orig_target = target;
    float orig_vel_limit = M1.velocity_limit;
    float orig_current_limit = peak_current_limit;
    float orig_cont_current_limit = continuous_current_limit; // <-- Store continuous limit
    float orig_following_error = max_following_error;

    // Set temporary "stall detection" limits
    Serial.println(F("Setting homing stall limits (High Current, Low Follow Error)..."));
    peak_current_limit = calibration_current_limit;
    M1.current_limit = calibration_current_limit; 
    continuous_current_limit = calibration_current_limit; // <-- Set continuous limit high
    max_following_error = calibration_following_error;  // <-- Set follow error low
    M1.controller = MotionControlType::angle; // <-- Use ANGLE mode
    M1.velocity_limit = calibration_velocity; 
    
    // --- Move to Negative End ---
    Serial.println(F("Moving to negative end stop..."));
    target = M1.shaft_angle; // Start ramp from current position
    resetDriveFault();
    last_run_us = micros();
    
    while (!drive_disabled) {
        dt_s = (micros() - last_run_us) * 1e-6f;
        if(dt_s < 0.0001f) continue; // 10kHz loop
        last_run_us = micros();

        target -= calibration_velocity * dt_s; // Ramp target negative
        M1.loopFOC();
        M1.move(target);
        checkSafetyPack(); 
    }
    float home_angle = M1.shaft_angle;
    Serial.print(F("Found home at [rad]: ")); Serial.println(home_angle);

    // Set new "zero" position
    M1.sensor->update();
    M1.sensor_offset = M1.sensor_direction * M1.sensor->getAngle();
    
    // Set new limits based on known stroke
    soft_limit_negative = 0.0f + calibration_stroke_buffer;
    soft_limit_positive = known_stroke_distance - calibration_stroke_buffer;

    if (soft_limit_positive <= soft_limit_negative) {
        Serial.println(F("WARN: Stroke buffer is too large! Disabling soft limits."));
        soft_limits_enabled = false;
    } else {
        soft_limits_enabled = true;
    }

    Serial.print(F("Homing complete. Limits set: "));
    Serial.print(soft_limit_negative); Serial.print(F(" to ")); Serial.println(soft_limit_positive);

    // --- Restore original settings ---
    Serial.println(F("Restoring original limits..."));
    M1.controller = orig_controller;
    target = 0; // Set target to 0 (new home)
    M1.move(0); 
    M1.velocity_limit = orig_vel_limit;
    peak_current_limit = orig_current_limit;
    M1.current_limit = orig_current_limit;
    continuous_current_limit = orig_cont_current_limit; // <-- Restore continuous limit
    max_following_error = orig_following_error;
    resetDriveFault();
}


/**
 * @brief Decides whether to run calibration or just homing based on globals.
 */
void setupSoftLimits() {
    if (run_calibration_on_startup) {
        calibrateAxis();
    } else if (known_stroke_distance > 0.01f) { // Check if a valid stroke is set
        homeAxis();
    } else {
        Serial.println(F("Soft limits disabled. Set 'run_calibration_on_startup' or 'known_stroke_distance'."));
    }
}

void setup(){
  SystemClock_Config();
  delay(1000);
  setupSerial();
  setupDRV8301(true); // 'true' for verbose setup, 'false' for quiet
  setupMotorParameters();
  setupMotorPIDs();
  linkMotorComponents();
  // Serial.println(F("Motor characterising..."));
  // M1.characteriseMotor(3.0f);
  Serial.println(F("Motor ready."));
  //setupSoftLimits();
  // SD1.init();
  // SD1.enableInterrupt(onStep);
  // SD1.attach(&target);
  delay(100);
}

void loop(){
  // --- Master Safety Check ---
  checkSafetyPack();
  if (drive_disabled) {
    M1.disable();
    digitalWrite(EN_GATE, LOW); // Disable DRV
    return; 
  }

  // --- Timed block for diagnostics ---
  if (loopcounter == loopiter){
    start = micros();
  }
  
  // --- Core FOC Loops ---
  M1.loopFOC();

  // --- Enforce Soft Limits ---
  if (soft_limits_enabled) {
      if (M1.controller == MotionControlType::angle) {
          // Constrain the angle target
          target = _constrain(target, soft_limit_negative, soft_limit_positive);
      } else if (M1.controller == MotionControlType::velocity) {
          // If at a limit, only allow movement away from it
          if (M1.shaft_angle >= soft_limit_positive && target > 0) {
              target = 0; // Stop positive movement
          } else if (M1.shaft_angle <= soft_limit_negative && target < 0) {
              target = 0; // Stop negative movement
          }
      }
      // Note: Torque mode will not be stopped by soft limits,
      // but will be stopped by the safety pack's following error (hard stop).
  }

  M1.move(target);
  
  // --- Timed block for sensors & safety ---
  if (loopcounter == loopiter){
    updateExternalSensors();
    finish = micros();
    looptime = (finish - start);
    loopcounter = 0;
  }
  loopcounter++;
}