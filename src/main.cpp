#include <SimpleFOC.h>
#include "encoders/stm32hwencoder/STM32HWEncoder.h"
#include "utilities/stm32pwm/STM32PWMInput.h" // <-- 1. ADD THIS INCLUDE
#include "pins.h"
#include "globals.h"
#include "DRV8301.h"
#include "current_sense/hardware_specific/stm32/stm32f4/stm32f4_hal.h" // Make sure to include your HAL header

// -- All Sensing Parameters in one place --
#define SHUNT_RESISTOR 0.0005f
#define CSA_GAIN 80.0f
#define PIN_CUR_B M0_IB
#define PIN_CUR_C M0_IC
#define PIN_VBUS PA6
#define VBUS_DIVIDER_RATIO 19.0f
#define PIN_TEMP_M1 PC5


// PWM Control Parameters
#define PIN_BRAKE_RESISTOR PB11
#define BRAKE_TARGET_VOLTAGE 56.5f // MUST BE GREATER THAN SUPPLY VOLTAGE
#define BRAKE_P_GAIN 0.1f         
#define BRAKE_I_GAIN 0.05f

// --- 2. ADD NEW DEFINES FOR PWM/DIR INPUT ---
// Choose any two available GPIO pins. PA2/PA3 are examples.
#define PIN_PWM_COMMAND PA_2_ALT2
#define PIN_DIR_COMMAND PA3
#define MAX_PWM_VELOCITY 314.15f // [rad/s] Velocity target at 100% duty cycle

// Use the constructor that includes Vbus information
LowsideCurrentSense CS1 = LowsideCurrentSense(SHUNT_RESISTOR, CSA_GAIN, _NC, PIN_CUR_B, PIN_CUR_C, PIN_VBUS, VBUS_DIVIDER_RATIO, PIN_TEMP_M1);

float current_bandwidth = 100; //Hz
float phase_resistance = 0.2816;
float d_phase_inductance = 0.00037;
float q_phase_inductance = 0.00037;
float VBUS_S = 0;
float Fet_Temp_M1 = 0;
float duty_cycle = 0;

// Motor instance
BLDCMotor M1 = BLDCMotor(5, phase_resistance, 145, q_phase_inductance);
BLDCDriver6PWM DR1 = BLDCDriver6PWM(M0_INH_A,M0_INL_A, M0_INH_B,M0_INL_B, M0_INH_C,M0_INL_C);
DRV8301 gate_driver = DRV8301(SPI3_MOSO, SPI3_MISO, SPI3_SCL, SPI3_CS, EN_GATE, nFAULT);
PhaseCurrent_s current1;
STM32HWEncoder E1 = STM32HWEncoder(16384, M0_ENC_A, M0_ENC_B, _NC);

// --- 3. INSTANTIATE THE PWM INPUT OBJECT ---
STM32PWMInput pwm_input(PIN_PWM_COMMAND, 200.0);

//StepDirListener SD1 = StepDirListener(PA2, PA3, 0.006135f);//2*PI/1024=); // For 4096 steps/rev encoder
// void onStep() { SD1.handle(); } 

void setup(){
  delay(10000);
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // --- 4. INITIALIZE PWM/DIR PINS ---
  pinMode(PIN_DIR_COMMAND, INPUT); // Set DIR pin as input
  pwm_input.initialize();                // Initialize hardware timer for PWM input
  Serial.println("PWM/Dir input ready.");


  DR1.pwm_frequency = 25000;
  // power supply voltage [V]
  DR1.voltage_power_supply = 56.0;
  // Max DC voltage allowed - default voltage_power_supply
  DR1.voltage_limit = 56.0;
  M1.motion_downsample = 10; // run the control loop at each foc loop
  M1.voltage_limit = 56.0;   // [V]
  
  // *** MODIFIED ***
  // Set the FOC hard limit to the absolute PEAK current
  M1.current_limit = peak_current_limit; // Amps
  
  M1.voltage_sensor_align = 5.0;
  M1.acceleration_limit = 500.0;

    // velocity PID controller parameters
  M1.PID_velocity.P = 0.05;
  M1.PID_velocity.I = 1.0;
  M1.PID_velocity.D = 0;
  M1.PID_velocity.output_ramp = 0;
  M1.LPF_velocity.Tf = 0;
   
  // angle PID controller 
  M1.P_angle.P = 20.0;
  M1.P_angle.I = 0;
  M1.P_angle.D = 0;
  M1.P_angle.output_ramp = 0;
  M1.LPF_angle.Tf = 0;

  M1.PID_current_q.P = q_phase_inductance*current_bandwidth*_2PI;
  M1.PID_current_q.I= M1.PID_current_q.P*phase_resistance/q_phase_inductance;
  M1.PID_current_d.P= d_phase_inductance*current_bandwidth*_2PI;
  M1.PID_current_d.I = M1.PID_current_d.P*phase_resistance/d_phase_inductance;
  M1.LPF_current_q.Tf = 1/(5.0*current_bandwidth); 
  M1.LPF_current_d.Tf = 1/(5.0*current_bandwidth);

  DR1.init();
  gate_driver.begin(PWM_INPUT_MODE_6PWM);
  gate_driver.set_csa_gain(CSA_GAIN_80);
  M1.linkDriver(&DR1);
  E1.init();
  M1.linkSensor(&E1);
  
  // control loop type and torque mode 
  M1.torque_controller = TorqueControlType::foc_current;
  M1.controller = MotionControlType::velocity; // <-- This is already set, which is correct!
  M1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  M1.velocity_limit = 9999; // [rad/s]



  // link the driver
  CS1.linkDriver(&DR1);
  CS1.linkMotor(&M1);
  // init the current sense
  CS1.init();
  CS1.initBrakeResistorPWM(PIN_BRAKE_RESISTOR, BRAKE_TARGET_VOLTAGE, BRAKE_P_GAIN, BRAKE_I_GAIN);
  CS1.skip_align = true;
  M1.linkCurrentSense(&CS1);
   // initialise motor
  M1.init();
  // init FOC  
  M1.initFOC(); 
  // SD1.init();
  // SD1.enableInterrupt(onStep);
  // SD1.attach(&target);

  delay(100);
}

void loop(){
  // --- 5. ADD PWM/DIR LOGIC AT THE TOP OF THE LOOP ---
  // This updates the 'target' variable on every single loop.
  
  // // Get the duty cycle (0.0f to 1.0f) from the hardware timer
  // duty_cycle = (pwm_input.getDutyCyclePercent())/100; 

  // // Read the direction pin
  // bool direction = digitalRead(PIN_DIR_COMMAND); // HIGH = forward, LOW = reverse (or vice-versa)

  // // Map duty cycle and direction to the target velocity
  // float new_target = duty_cycle * MAX_PWM_VELOCITY;
  // if (direction == LOW) { // You can swap LOW/HIGH to reverse direction
  //     new_target = -new_target;
  // }
  
  // target = new_target; // Update the global target


  // --- Master Safety Check ---
  if (drive_disabled) {
    // If a fault occurred, keep the motor and driver disabled and do nothing else.
    M1.disable();
    digitalWrite(EN_GATE, LOW); // Assumes EN_GATE is the enable pin
    return; 
  }
  // --- End Safety Check ---


  if (loopcounter == loopiter){
    start = micros();
  }
  // foc loop
  M1.loopFOC();
  
  // motion control
  M1.move(target); // <-- This line now uses the 'target' you just set from the PWM input
  
  // // monitoring 
  // M1.monitor();
  // // user communication
  // command.run();
  
  if (loopcounter == loopiter){
    //Loop time finish 
    current1 = CS1.getPhaseCurrents();
    VBUS_S = CS1.getVbusVoltage();
    CS1.updateBrakeResistor();
    CS1.checkTemperature();
    Fet_Temp_M1 = CS1.getTemperature();
    finish = micros();
    looptime = (finish - start);
    
    // --- Safety Pack Checks ---
    
    // 1. Current Limit Check
    // Get the FOC current magnitude
    float current_magnitude = _sqrt(M1.current.q * M1.current.q + M1.current.d * M1.current.d);

    if (current_magnitude > continuous_current_limit) {
        // Current is in the "peak" zone (above continuous)
        if (peak_current_start_time_ms == 0) {
            // Just entered the peak zone, start the timer
            peak_current_start_time_ms = millis();
        } else if (millis() - peak_current_start_time_ms > peak_current_timeout_ms) {
            // Been in the peak zone for too long, disable the drive
            M1.disable();
            drive_disabled = true;
            Serial.println("ERROR: Sustained current over continuous limit! Drive disabled.");
        }
    } else {
        // Current is at or below the continuous limit, all good. Reset timer.
        peak_current_start_time_ms = 0;
    }


    // 2. Following Error Check
    float following_error = 0;
    if (M1.controller == MotionControlType::angle) {
        following_error = target - M1.shaft_angle; // 'target' would be angle
    } else if (M1.controller == MotionControlType::velocity) {
        following_error = target - M1.shaft_velocity; // 'target' is velocity
    }

    if (M1.controller != MotionControlType::torque) { // Don't check for torque control
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
    // --- End Safety Pack Checks ---

    loopcounter = 0;
   
  }
  loopcounter++;
}