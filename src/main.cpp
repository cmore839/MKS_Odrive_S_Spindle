#include <SimpleFOC.h>
#include "encoders/stm32hwencoder/STM32HWEncoder.h"
#include "pins.h"
#include "globals.h"
#include "DRV8301.h"
#include "current_sense/hardware_specific/stm32/stm32f4/stm32f4_hal.h" // Make sure to include your HAL header
#define PIN_VBUS PA6
#define VBUS_DIVIDER_RATIO 19.0f // (18k + 1k) / 1k

float current_bandwidth = 330; //Hz
float phase_resistance = 0.2816;
float d_phase_inductance = 0.00037;
float q_phase_inductance = 0.00037;
float VBUS_S = 0;

// Motor instance
BLDCMotor M1 = BLDCMotor(5, phase_resistance, 145, q_phase_inductance);
BLDCDriver6PWM DR1 = BLDCDriver6PWM(M0_INH_A,M0_INL_A, M0_INH_B,M0_INL_B, M0_INH_C,M0_INL_C);
// DRV8301 gate_driver = DRV8301(MOSI, MISO, SCLK, CS, EN_GATE, FAULT);
DRV8301 gate_driver = DRV8301(SPI3_MOSO, SPI3_MISO, SPI3_SCL, SPI3_CS, EN_GATE, nFAULT);
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&M1, cmd); }
LowsideCurrentSense CS1 = LowsideCurrentSense(0.0005f, 80.0f, _NC, M0_IB, M0_IC, PIN_VBUS, VBUS_DIVIDER_RATIO);
PhaseCurrent_s current1;
STM32HWEncoder E1 = STM32HWEncoder(16384, M0_ENC_A, M0_ENC_B, _NC);

void setup(){
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  DR1.pwm_frequency = 20000;
  // power supply voltage [V]
  DR1.voltage_power_supply = 24;
  // Max DC voltage allowed - default voltage_power_supply
  DR1.voltage_limit = 24;
  M1.motion_downsample = 100; // run the control loop at each foc loop
  M1.voltage_limit = 12;   // [V]
  M1.current_limit = 0.5; // Amps
  M1.velocity_limit = 2; // [rad/s]
  M1.voltage_sensor_align = 0.5;

    // velocity PID controller parameters
  M1.PID_velocity.P = 0.05;
  M1.PID_velocity.I = 5.0;
  M1.PID_velocity.D = 0;
  M1.PID_velocity.output_ramp = 0.0;
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
  
  // comment out if not needed
  M1.useMonitoring(Serial);
  M1.monitor_variables = _MON_CURR_Q | _MON_CURR_D;
  M1.monitor_downsample = 1000;

  // add target command T
  command.add('M', doMotor, "motor M0");

  // initialise motor
  M1.init();

  // link the driver
  CS1.linkDriver(&DR1);
  // init the current sense
  CS1.init();  
  CS1.skip_align = true;
  M1.linkCurrentSense(&CS1);
  
  // init FOC  
  M1.initFOC(); 
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
  // monitoring 
  M1.monitor();
  // user communication
  command.run();
  if (loopcounter == loopiter){
    //Loop time finish 
    finish = micros();
    looptime = (finish - start);
    current1 = CS1.getPhaseCurrents();
    loopcounter = 0;
    VBUS_S = CS1.getVbusVoltage();
  }
  loopcounter++;
}