/*
(0°C × 9/5) + 32 = 32°F

Calculate Suction Superheat (SPH) with the following formula: 
Suction Superheat (SPH) = Actual Suction Gas Temperature – Temperature from PT Chart. 
*/

uint16_t registers[10];

int16_t temps[6]; // For 1 ADAM Module
int16_t holdingRegisters[50]; // For CoolMay HMI

// Compressor data structure
struct CompX {
  int16_t SuctionTemp = 0;
  int16_t dischargeTemp = 0;
  int16_t SprayTemp = 0;
  int16_t SuperHeatPV = 0;
  int16_t SupplyTemp = 0;
  int16_t ReturnTemp = 0;
  uint16_t SuperHeatSV = 0;

  float suctionPressure = 0;
  float dischargePressure = 0;

  bool comp_state = 0;
  uint16_t comp_hrs_sv = 0;
  uint16_t comp_fan_sv = 0;
  uint16_t counterHr = 0;

  uint16_t Kp = 0;
  uint16_t Ki = 0;
  uint16_t Kd = 0;
  uint16_t UpperLimit = 0;
  uint16_t LowerLimit = 0;

  uint16_t vfd_freq = 0;
  uint16_t acc_time;
  uint16_t dec_time;
  uint16_t out_voltage = 0;
  uint16_t out_amps = 0;
  uint16_t bus_voltage = 0;
  uint16_t out_freq = 0;

  uint16_t suc_temp_low_alarm = 0;
  uint16_t suc_temp_high_alarm = 0;
  uint16_t dis_temp_low_alarm = 0;
  uint16_t dis_temp_high_alarm = 0;
  uint16_t spray_temp_low_alarm = 0;
  uint16_t spray_temp_high_alarm = 0;
  uint16_t suc_pres_low_alarm = 0;
  uint16_t suc_pres_high_alarm = 0;
  uint16_t dis_pres_low_alarm = 0;
  uint16_t dis_pres_high_alarm = 0;
  uint16_t superheat_low_alarm = 0;
  uint16_t superheat_high_alarm = 0;
  uint16_t suc_sensor_disconnect_alarm = 0;
  uint16_t dis_sensor_disconnect_alarm = 0;
  uint16_t spray_sensor_disconnect_alarm = 0;

  uint16_t suc_temp_low_SV = 0;
  uint16_t suc_temp_high_SV = 0;
  uint16_t dis_temp_low_SV = 0;
  uint16_t dis_temp_high_SV = 0;
  uint16_t spray_temp_low_SV = 0;
  uint16_t spray_temp_high_SV = 0;
  uint16_t suc_pres_low_SV = 0;
  uint16_t suc_pres_high_SV = 0;
  uint16_t dis_pres_low_SV = 0;
  uint16_t dis_pres_high_SV = 0;
  uint16_t superheat_low_SV = 0;
  uint16_t superheat_high_SV = 0;
};
// Compressor Comp1,Comp2;  // compressor # 01


struct ExpansionValve {
  uint16_t exv_debug_steps = 0;
  uint16_t exv_step_delay = 0;
  uint16_t exv_total_steps = 0;
  uint16_t currentStep = 0;  // exv_total_steps = MAX_STEPS of EXV to Open/Close
  int pulseEXV = 0;
  bool stepDone = false;        // EXV one step complete flag
  bool move_direction = false;  // false for Close and true for Open direction
};
ExpansionValve EXV;

// enum init_exv : uint8_t {
//   reset,
//   initializing,
//   idle,
//   ready
// };
// init_exv exv_state = idle;

// float Setpoint = 0, Input = 0, Output = 0;
float kii = 0, kpp = 0, kdd = 0;

// for both compressors
uint16_t startSW, stopSW, resetSW = 0;
float cwi_Temp, cwo_sv_temp, cwo_Temp = 0;

uint16_t* floatAsRegisters;
unsigned long previousMillis = 0;
unsigned long PIDmillis = 0;
// unsigned long write_exv_delay = 0;

uint16_t counter, counterMin, solenoid_SVL_enable, Chiller_ondelay, ondelay_timer, alarm_delay = 0;
bool startMachineFlag = false, startingDelay = false, Chiller_ondelay_led = 0;

// Alarms - not used
uint16_t cwot_sensor_disconnect_alarm, cwit_sensor_disconnect_alarm, loader_SVL_time, UnLoader_SVL_time = 0;
