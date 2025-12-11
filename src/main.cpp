#include "globals.h"

// Instantiate SPIClass for SPI3 using pins from pins.h
SPIClass spi3(SPI3_MOSO, SPI3_MISO, SPI3_SCL);

// --- Component Instances ---
LowsideCurrentSense CS1 = LowsideCurrentSense(SHUNT_RESISTOR, CSA_GAIN, _NC, PIN_CUR_B, PIN_CUR_C, PIN_VBUS, VBUS_DIVIDER_RATIO, PIN_TEMP_M1);
BLDCMotor M1 = BLDCMotor(2, phase_resistance, kv_rating, q_phase_inductance);
BLDCDriver6PWM DR1 = BLDCDriver6PWM(M0_INH_A,M0_INL_A, M0_INH_B,M0_INL_B, M0_INH_C,M0_INL_C);
STM32HWEncoder E1 = STM32HWEncoder(1110, M0_ENC_A, M0_ENC_B, _NC);


// ===================================================================
//
//                      HELPER FUNCTIONS
//
// ===================================================================

void setupSerial() {

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
  DR1.voltage_power_supply = 12.0;
  DR1.voltage_limit = 12.0;
  
  E1.min_elapsed_time = 0.00005; // 50 us for 20kHz encoder update rate

  M1.motion_downsample = 10; // run the control loop at each foc loop
  M1.voltage_limit = 12.0;   // [V]
  M1.current_limit = peak_current_limit; // FOC hard limit is the absolute PEAK
  M1.voltage_sensor_align = 5.0;
  M1.acceleration_limit = 5000.0;
  M1.velocity_limit = 50; // [rad/s]

  M1.torque_controller = TorqueControlType::foc_current;
  M1.controller = MotionControlType::angle;
  M1.foc_modulation = FOCModulationType::SpaceVectorPWM;
}

void setupMotorPIDs() {
  // velocity PID controller parameters
  M1.PID_velocity.P = 0.04;
  M1.PID_velocity.I = 0.0;
  M1.PID_velocity.D = 0;
  M1.PID_velocity.output_ramp = 0;
  M1.LPF_velocity.Tf = 0;
   
  // angle PID controller 
  M1.P_angle.P = 100.0;
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
  CS1.linkMotor(&M1);
  CS1.init();
  CS1.initBrakeResistorPWM(PIN_BRAKE_RESISTOR, BRAKE_TARGET_VOLTAGE, BRAKE_P_GAIN, BRAKE_I_GAIN);
  CS1.skip_align = true;
  M1.linkCurrentSense(&CS1);

  // Initialise motor and FOC
  M1.init();
  M1.initFOC(); 
}

/**
 * @brief Read all external sensors (Vbus, Temp, etc.)
 */
void updateExternalSensors() {
    current1 = CS1.getPhaseCurrents();
    VBUS_S = CS1.getVbusVoltage();
    CS1.updateBrakeResistor();
    CS1.checkTemperature();
    Fet_Temp_M1 = CS1.getTemperature();
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
    if (Fet_Temp_M1 > max_fet_temp) {
        M1.disable();
        drive_disabled = true;
        Serial.println("ERROR: FET Over-temperature! Drive disabled.");
    }
}


// ===================================================================
//
//                      SETUP & LOOP
//
// ===================================================================

void setup(){
  SystemClock_Config();
  delay(1000);
  
  setupSerial();
  setupDRV8301(true); // 'true' for verbose setup, 'false' for quiet
  setupMotorParameters();
  setupMotorPIDs();
  linkMotorComponents();

  Serial.println(F("Motor characterising..."));
  M1.characteriseMotor(3.0f);
  Serial.println(F("Motor ready."));

  delay(100);
}

void loop(){
  // --- Master Safety Check ---
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
  M1.move(target);
  
  // --- Timed block for sensors & safety ---
  if (loopcounter == loopiter){
    finish = micros();
    looptime = (finish - start);
    
    updateExternalSensors();
    checkSafetyPack();
    
    loopcounter = 0;
  }
  loopcounter++;
}