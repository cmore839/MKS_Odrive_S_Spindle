#include <SimpleFOC.h>
#include "encoders/stm32hwencoder/STM32HWEncoder.h"
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
#define PIN_TEMP_M1 PC5 // New temperature sensor pin


// PWM Control Parameters
#define PIN_BRAKE_RESISTOR PB11
#define BRAKE_TARGET_VOLTAGE 56.5f // MUST BE GREATER THAN SUPPLY VOLTAGE
#define BRAKE_P_GAIN 0.1f         
#define BRAKE_I_GAIN 0.05f

// Use the constructor that includes Vbus information
LowsideCurrentSense CS1 = LowsideCurrentSense(SHUNT_RESISTOR, CSA_GAIN, _NC, PIN_CUR_B, PIN_CUR_C, PIN_VBUS, VBUS_DIVIDER_RATIO, PIN_TEMP_M1);

float current_bandwidth = 100; //Hz
float phase_resistance = 0.2816;
float d_phase_inductance = 0.00037;
float q_phase_inductance = 0.00037;
float VBUS_S = 0;
float Fet_Temp_M1 = 0;

// Motor instance
BLDCMotor M1 = BLDCMotor(5, phase_resistance, 145, q_phase_inductance);
BLDCDriver6PWM DR1 = BLDCDriver6PWM(M0_INH_A,M0_INL_A, M0_INH_B,M0_INL_B, M0_INH_C,M0_INL_C);
DRV8301 gate_driver = DRV8301(SPI3_MOSO, SPI3_MISO, SPI3_SCL, SPI3_CS, EN_GATE, nFAULT);
PhaseCurrent_s current1;
STM32HWEncoder E1 = STM32HWEncoder(16384, M0_ENC_A, M0_ENC_B, _NC);

// StepDirListener SD1 = StepDirListener(PA2, PA3, 0.006135f);//2*PI/1024=); // For 4096 steps/rev encoder
// void onStep() { SD1.handle(); } 

void setup(){
  delay(10000);
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  DR1.pwm_frequency = 25000;
  // power supply voltage [V]
  DR1.voltage_power_supply = 56.0;
  // Max DC voltage allowed - default voltage_power_supply
  DR1.voltage_limit = 56.0;
  M1.motion_downsample = 10; // run the control loop at each foc loop
  M1.voltage_limit = 56.0;   // [V]
  M1.current_limit = 2.0; // Amps
  M1.voltage_sensor_align = 5.0;

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
  M1.controller = MotionControlType::velocity;
  M1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  M1.velocity_limit = 9999; // [rad/s]

 // initialise motor
  M1.init();

  // link the driver
  CS1.linkDriver(&DR1);
  CS1.linkMotor(&M1);
  // init the current sense
  CS1.init();
  CS1.initBrakeResistorPWM(PIN_BRAKE_RESISTOR, BRAKE_TARGET_VOLTAGE, BRAKE_P_GAIN, BRAKE_I_GAIN);
  CS1.skip_align = true;
  M1.linkCurrentSense(&CS1);
  
  // init FOC  
  M1.initFOC(); 
  // SD1.init();
  // SD1.enableInterrupt(onStep);
  // SD1.attach(&target);

  delay(100);
}

void loop(){
  if (loopcounter == loopiter){
    start = micros();
  }
  // foc loop
  M1.loopFOC();
  // motion control
  M1.move(target);
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
    loopcounter = 0;
   
  }
  loopcounter++;
}