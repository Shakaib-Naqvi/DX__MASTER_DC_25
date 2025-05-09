#define DEBUG
#define RX_BUFFER_SIZE 256

#include <Wire.h>
#include <PID_v1_bc.h>  // https://github.com/drf5n/Arduino-PID-Library
#include <Preferences.h>
#include "variables.h"
#include "definitions.h"
#include "modbus_rtu.h"
#include "classes.h"

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Preferences preferences;

TemperatureSensor tempSensors;
// PressureSensor pressureSensor1, pressureSensor2;
CompX Comp1, Comp2;
SolenoidValve valve1, valve2;

#include "IO.h"
#include "Temp.h"
#include "ads1115.h"
#include "pt_chart.h"
#include "vfd.h"
#include "coolmay_hmi_rs485.h"

// void scanSensors() {  // scans VFD Parameters , ADAM-4015 Temp module and Pressure ADCs and compares the vales within set ranges
//   scanADC();          // calculates Discharge and Suction Pressures
//   CompX.suctionPressure = suction + 2;
//   CompX.dischargePressure = discharge + 5;

//   uint16_t tempReg[6] = { 0, 0, 0, 0, 0, 0 }, VFD_Reg[5] = { 0, 0, 0, 0, 0 };  // from ADAM-4015 module
//   if (readHoldingRegisters(VFD_1_ID, 0x3000, 5, VFD_Reg)) {
//     CompX.out_freq = VFD_Reg[0];     //40011 - OUTPUT FREQ
//     CompX.out_voltage = VFD_Reg[3];  //40012 - OUTPUT VOLTAGE
//     CompX.out_amps = VFD_Reg[4];     //40013 - OUTPUT AMPS
//     CompX.bus_voltage = VFD_Reg[2];  //40016 - BUS VOLTAGE
//   } else {
//     Serial.println("VFD # 1 Communication Error!");
//   }

//   if (readHoldingRegisters(RTD_SLAVE_ID_1, 0, 6, tempReg)) {
//     for (int i = 0; i < 6; i++) {
//       Serial.println(tempReg[i]);
//     }

//   } else {
//     Serial.println("Adam-4015 # 1 Communication Error!");
//   }

//   // uint16_t Sat_Temp = Temp_from_PT_chart(CompX.suctionPressure);  // Saturation Temp from PT chart
//   // CompX.SuperHeatPV = CompX.SuctionTemp - Sat_Temp;               // if Superheat == actual suction temp that means suction pressure sensor is disconnected
// }


void runVFD(uint16_t freq, uint16_t command) {
  switch (command) {
    case 0:  // stop operation
      // Output_Write(CompMotor, 0);
      writeSingleRegister(VFD_1_ID, Control_Address, VFD_Stop);
      Serial.println("VFD Stop");
      break;
    case 1:  // run oepration
      // Output_Write(Condenser_Fan1, 1);
      writeSingleRegister(VFD_1_ID, Freq_Address, freq);  // Set frequency to 50.00 Hz (50.00 * 200 = 10000)
      writeSingleRegister(VFD_1_ID, Control_Address, VFD_Run);
      Serial.println("Set frequency to 50.00 Hz");
      Serial.println("VFD Run");

      // Output_Write(CompMotor, 1);
      break;
    default:
      Serial.println("Invalid VFD Command !");
      break;
  }
}


// #ifdef DEBUG
// void scanI2C() {
//   byte error, address;
//   int nDevices = 0;

//   Serial.println("Scanning...");

//   for (address = 1; address < 127; address++) {
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();

//     if (error == 0) {
//       Serial.print("I2C device found at address 0x");
//       if (address < 16)
//         Serial.print("0");
//       Serial.println(address, HEX);

//       nDevices++;=-//     }0
//   }
//   if (nDevices == 0)
//     Serial.println("No I2C devices found\n");
//   else
//     Serial.println("done\n");
// }
// #endif

// Temperature measurement task
// void temperatureTask(void* pvParameter) {
//   while (true) {
//     // Read temperature
//     sl_temp = readDS18B20Temperature(26);


//     d_temp = readDS18B20Temperature(14);


//     sp_temp = readDS18B20Temperature(27);


//     sa_temp = readDS18B20Temperature(25);


//     ra_temp = readDS18B20Temperature(25);


//     // Update last_read_time
//     last_read_time = millis();

//     uint16_t* floatAsRegisters = (uint16_t*)&sl_temp;
//     holdingRegisters[0] = floatAsRegisters[0];
//     holdingRegisters[1] = floatAsRegisters[1];

//     floatAsRegisters = (uint16_t*)&d_temp;
//     holdingRegisters[2] = floatAsRegisters[0];
//     holdingRegisters[3] = floatAsRegisters[1];

//     floatAsRegisters = (uint16_t*)&sp_temp;
//     holdingRegisters[4] = floatAsRegisters[0];
//     holdingRegisters[5] = floatAsRegisters[1];

//     floatAsRegisters = (uint16_t*)&sa_temp;
//     holdingRegisters[6] = floatAsRegisters[0];
//     holdingRegisters[7] = floatAsRegisters[1];

//     floatAsRegisters = (uint16_t*)&ra_temp;
//     holdingRegisters[8] = floatAsRegisters[0];
//     holdingRegisters[9] = floatAsRegisters[1];

//     // sl_temp = (sl_temp * 1.8) + 32;
//     // d_temp = (d_temp * 1.8) + 32;
//     // sp_temp = (sp_temp * 1.8) + 32;
//     // sa_temp = (sa_temp * 1.8) + 32;
//     // ra_temp = (ra_temp * 1.8) + 32;


//     sl_pres = (sl_temp * 1.8) + 32;  // Convert to Fahrenheit
//     sl_pres = (d_temp * 1.8) + 32;   // Convert to Fahrenheit

//     // sl_pres = get_pressure_from_temp(sl_temp, gas_selected_s);
//     // dl_pres = get_pressure_from_temp(d_temp, gas_selected_d);

//     sl_pres = get_pressure_from_temp(sl_temp, 7);
//     dl_pres = get_pressure_from_temp(d_temp, 5);

//     floatAsRegisters = (uint16_t*)&sl_pres;
//     holdingRegisters[17] = floatAsRegisters[0];
//     holdingRegisters[18] = floatAsRegisters[1];

//     floatAsRegisters = (uint16_t*)&dl_pres;
//     holdingRegisters[19] = floatAsRegisters[0];
//     holdingRegisters[20] = floatAsRegisters[1];




//     // writeSingleRegister(0x00, sl_temp);
//     // writeSingleRegister(0x01, d_temp);
//     // writeSingleRegister(0x02, sp_temp);
//     // writeSingleRegister(0x03, sa_temp);
//     // writeSingleRegister(0x04, ra_temp);

//     vTaskDelay(500 / portTICK_PERIOD_MS);  // Delay for 500 milliseconds
//   }
// }


// HMI Writing task
void HMITask(void* pvParameter) {
  while (true) {
    save_in_holding_registers();
    writeMultipleRegisters(1, 0, holdingRegisters, 50);

    vTaskDelay(500 / portTICK_PERIOD_MS);  // Delay for 500 milliseconds
  }
}

void save_in_holding_registers() {

  holdingRegisters[0] = Comp1.SuctionTemp;
  holdingRegisters[1] = Comp1.dischargeTemp;
  holdingRegisters[2] = Comp1.SprayTemp;
  holdingRegisters[3] = Comp1.SupplyTemp;
  holdingRegisters[4] = Comp2.SuctionTemp;
  holdingRegisters[5] = Comp2.dischargeTemp;
  holdingRegisters[6] = currentStep;
  holdingRegisters[7];
  holdingRegisters[8];
  holdingRegisters[9];
  holdingRegisters[10];

  // holdingRegisters[11] = r_a_temp_Setpoint;
  // float_As_Registers = (uint16_t*)&Suction_Super_Heat;
  // holdingRegisters[12] = float_As_Registers[0];
  // holdingRegisters[13] = float_As_Registers[1];
  // holdingRegisters[14] = compressor_onoff;
  // holdingRegisters[15] = sv_l_u;
  // holdingRegisters[16] = condenser_fan_1;
  // holdingRegisters[17] = 0;
  // holdingRegisters[18] = 0;
  // holdingRegisters[19] = 0;
  // holdingRegisters[20] = 0;
  // holdingRegisters[21] = sl_ps;
  // holdingRegisters[22] = dl_ps;
  // holdingRegisters[23] = o_ps;
  // holdingRegisters[24] = pf_relay;
  // holdingRegisters[25] = Low_Temperature_Alarm;
  // holdingRegisters[26] = High_Temperature_Alarm;
  // holdingRegisters[27] = Spray_Temperature_Alarm;
  // holdingRegisters[28] = 0;
  // holdingRegisters[29] = 0;
  // holdingRegisters[30] = 0;
  // holdingRegisters[31] = 0;
  // holdingRegisters[32] = 0;
  // holdingRegisters[33] = 0;
  // holdingRegisters[34] = start;
  // holdingRegisters[35] = 0;
  // holdingRegisters[36] = compressor_on_delay;
  // switch (gas_selected_s) {
  //   case 0:
  //     holdingRegisters[37] = 1;
  //     holdingRegisters[38] = 0;
  //     holdingRegisters[39] = 0;
  //     holdingRegisters[40] = 0;
  //     holdingRegisters[41] = 0;
  //     break;
  //   case 1:
  //     holdingRegisters[37] = 0;
  //     holdingRegisters[38] = 1;
  //     holdingRegisters[39] = 0;
  //     holdingRegisters[40] = 0;
  //     holdingRegisters[41] = 0;
  //     break;
  //   case 2:
  //     holdingRegisters[37] = 0;
  //     holdingRegisters[38] = 0;
  //     holdingRegisters[39] = 1;
  //     holdingRegisters[40] = 0;
  //     holdingRegisters[41] = 0;
  //     break;
  //   case 3:
  //     holdingRegisters[37] = 0;
  //     holdingRegisters[38] = 0;
  //     holdingRegisters[39] = 0;
  //     holdingRegisters[40] = 1;
  //     holdingRegisters[41] = 0;
  //     break;
  //   case 4:
  //     holdingRegisters[37] = 0;
  //     holdingRegisters[38] = 0;
  //     holdingRegisters[39] = 0;
  //     holdingRegisters[40] = 0;
  //     holdingRegisters[41] = 1;
  //     break;
  //   default:
  //     holdingRegisters[37] = 1;
  //     holdingRegisters[38] = 0;
  //     holdingRegisters[39] = 0;
  //     holdingRegisters[40] = 0;
  //     holdingRegisters[41] = 0;
  //     break;
  // }
  // // holdingRegisters[42] = 0;
  // // holdingRegisters[43] = 0;
  // // holdingRegisters[44] = 0;
  // // holdingRegisters[45] = 0;
  // // holdingRegisters[46] = defrosting_on_delay;
  // // holdingRegisters[47] = defrosting_off_delay;
  // // holdingRegisters[48] = defrosting_onoff;
  // // if (defrosting_on_flag == 1) {
  // //   holdingRegisters[49] = remaining_time / 3600;
  // //   holdingRegisters[63] = (remaining_time % 3600) / 60;
  // //   holdingRegisters[64] = (remaining_time % 3600) % 60;
  // // } else {
  // //   holdingRegisters[49] = remaining_time / 3600;
  // //   holdingRegisters[63] = (remaining_time % 3600) / 60;
  // //   holdingRegisters[64] = (remaining_time % 3600) % 60;
  // // }
  // holdingRegisters[50] = totalInterruptCounter;
  // // if (sv_l_u != 1) {
  // holdingRegisters[51] = totalInterruptCounter_2;
  // // } else {
  // //   holdingRegisters[51] = 0;
  // // }
  // switch (gas_selected_d) {
  //   case 5:
  //     holdingRegisters[52] = 1;
  //     holdingRegisters[53] = 0;
  //     holdingRegisters[54] = 0;
  //     holdingRegisters[55] = 0;
  //     holdingRegisters[56] = 0;
  //     break;
  //   case 6:
  //     holdingRegisters[52] = 0;
  //     holdingRegisters[53] = 1;
  //     holdingRegisters[54] = 0;
  //     holdingRegisters[55] = 0;
  //     holdingRegisters[56] = 0;
  //     break;
  //   case 7:
  //     holdingRegisters[52] = 0;
  //     holdingRegisters[53] = 0;
  //     holdingRegisters[54] = 1;
  //     holdingRegisters[55] = 0;
  //     holdingRegisters[56] = 0;
  //     break;
  //   case 8:
  //     holdingRegisters[52] = 0;
  //     holdingRegisters[53] = 0;
  //     holdingRegisters[54] = 0;
  //     holdingRegisters[55] = 1;
  //     holdingRegisters[56] = 0;
  //     break;
  //   case 9:
  //     holdingRegisters[52] = 0;
  //     holdingRegisters[53] = 0;
  //     holdingRegisters[54] = 0;
  //     holdingRegisters[55] = 0;
  //     holdingRegisters[56] = 1;
  //     break;
  //   default:
  //     holdingRegisters[52] = 1;
  //     holdingRegisters[53] = 0;
  //     holdingRegisters[54] = 0;
  //     holdingRegisters[55] = 0;
  //     holdingRegisters[56] = 0;
  //     break;
  // }
  // // if (defrosting_en == 0) {
  // //   holdingRegisters[61] = 0;
  // //   holdingRegisters[62] = 1;

  // // } else {
  // //   holdingRegisters[61] = 1;
  // //   holdingRegisters[62] = 0;
  // // }
  // // if (defrosting_sw_timer == 1) {
  // //   holdingRegisters[65] = 1;
  // //   holdingRegisters[66] = 0;
  // // } else {
  // //   holdingRegisters[65] = 0;
  // //   holdingRegisters[66] = 1;
  // // }
  // // holdingRegisters[67] = tolerance;
  // // holdingRegisters[68] = Heat_Setpoint;
  // holdingRegisters[69] = compressor_off_delay;
  // holdingRegisters[71] = power_on_pres;
  // holdingRegisters[72] = cut_off_pres;
  // holdingRegisters[73] = solenoid_valve_on_delay;
  // // holdingRegisters[74] = defrosting_electrical;
  // // holdingRegisters[75] = defrosting_mechanical;

  // holdingRegisters[76] = condenser_fan_enable;
  // holdingRegisters[77] = condenser_fan_disable;
  // holdingRegisters[78] = low_temp_alarm;
  // // holdingRegisters[79] = high_temp_alarm;
  // // holdingRegisters[80] = low_pres_alarm;
  // // holdingRegisters[81] = high_pres_alarm;
  // // holdingRegisters[82] = stop_alarm;
  // // holdingRegisters[83] = 1; // inner_fan_alarm
  // // holdingRegisters[87] = saved_hours;
  // holdingRegisters[89] = stop;
  // holdingRegisters[90] = vfd_start;
}

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {  //Defining Interrupt function with IRAM_ATTR for faster access
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR onTimer_2() {  //Defining Interrupt function with IRAM_ATTR for faster access
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter_2++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

bool safety_check() {

#ifdef DEBUG
  if (pf_relay == 1) {
    Serial.println("Power Failure is 1");
  }
  if (sl_ps == 1) {
    Serial.println("Suction Line Pressure Switch is 1");
  }
  if (dl_ps == 1) {
    Serial.println("Discharge Line Pressure Switch is 1");
  }
  if (o_ps == 1) {
    Serial.println("Oil Pressure Switch is 1");
  }
  if (sl_temp < Low_Temperature_Alarm) {
    Serial.println("Suction Line Temperature < 20");
  }
  if (d_temp > High_Temperature_Alarm) {
    Serial.println("Discharge Line Temperature > 150");
  }
  if (sp_temp > Spray_Temperature_Alarm) {
    Serial.println("Spray Temperature > 50");
  }
  if (sl_pres < Low_Pressure_Alarm) {
    Serial.println("Suction Line Pressure < 20");
  }
  if (dl_pres > High_Pressure_Alarm) {
    Serial.println("Discharge Line Pressure > 600");
  }
#endif
  // if (inner_fan_sw != inner_fan) {
  //     inner_fan_alarm = false;
  //   } else {
  //     inner_fan_alarm = true;
  //   }
  if (sl_temp < Low_Temperature_Alarm) {
    low_temp_alarm = false;
  } else {
    low_temp_alarm = true;
  }
  if (d_temp > High_Temperature_Alarm) {
    high_temp_alarm = false;
  } else {
    high_temp_alarm = true;
  }
  if (pf_relay == 1 || sl_ps == 1 || dl_ps == 1 || o_ps == 1 || sl_temp < Low_Temperature_Alarm || d_temp > High_Temperature_Alarm || sp_temp > Spray_Temperature_Alarm || sl_pres < Low_Pressure_Alarm || dl_pres > High_Pressure_Alarm) {
    return false;
  } else {
    return true;
  }
}



void off_compressor() {
#ifdef DEBUG
  Serial.println("Counter Completed for Turning OFF Compressor");
  Serial.println("Turned OFF Compressor");
#endif
  compressor_onoff = 0;
  totalInterruptCounter = 0;
  timerAlarmDisable(Comp_on_off_timer);
  timer1_flag = false;
  timerWrite(Comp_on_off_timer, 0);
}


void on_compressor() {
#ifdef DEBUG
  Serial.println("Counter Completed for Compressor");
  Serial.println("Compressor Turned On");
#endif
  compressor_timer_flag = false;
  compressor_onoff = 1;
  totalInterruptCounter = 0;
  timerAlarmDisable(Comp_on_off_timer);
  timerWrite(Comp_on_off_timer, 0);
}

void on_solenoid() {
#ifdef DEBUG
  Serial.println("Counter Completed for Solenoid");
  Serial.println("Solenoid Valve Turned On");
#endif
  sv_l_u = 1;
  solenoid_on_timer_flag = false;
  totalInterruptCounter_2 = 0;
  timerAlarmDisable(solenoid_valve_timer);
  timerWrite(solenoid_valve_timer, 0);
}

void setup() {

  Serial.begin(115200);

  Wire.begin();



  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);  // Start in receive mode

  RS485Serial.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  // delay(1000);

  pinMode(EXV_1, OUTPUT);
  pinMode(EXV_2, OUTPUT);
  pinMode(EXV_3, OUTPUT);
  pinMode(EXV_4, OUTPUT);

#ifdef DEBUG
  Serial.println("Modbus RTU over RS485 initialized.");
#endif

  digitalWrite(EXV_1, HIGH);
  digitalWrite(EXV_2, HIGH);
  digitalWrite(EXV_3, HIGH);
  digitalWrite(EXV_4, HIGH);

  // digitalWrite(13, LOW);
  // scanI2C();

  // initads();

  // initializePCF8574(In_ADDRESS, Out_ADDRESS);

  // RS485Serial.setRxBufferSize(RX_BUFFER_SIZE);

  // RS485Serial.begin(RS485_BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  // RS485Serial.begin(RS485_BAUD_RATE, SERIAL_8E1, RS485_RX_PIN, RS485_TX_PIN, true, 512);
  // pinMode(RS485_DE_RE_PIN, OUTPUT);

  // digitalWrite(RS485_DE_RE_PIN, LOW);  // Set DE/RE pin low for receive mode

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, MAX_STEPS);
  myPID.SetSampleTime(1000);

  last_read_time = millis();

  xTaskCreatePinnedToCore(HMITask, "hmiTask", 2048, NULL, 1, NULL, 0);

  preferences.begin("EXV_Steps", false);
  currentStep = preferences.getInt("currentStep", 50);
  preferences.end();

  write_on_comp = millis();

  Comp_on_off_timer = timerBegin(0, 80, true);                   // timer 0, prescalar: 80, UP counting
  solenoid_valve_timer = timerBegin(1, 80, true);                // timer 1, prescalar: 80, UP counting
  timerAttachInterrupt(Comp_on_off_timer, &onTimer, true);       // Attach interrupt
  timerAttachInterrupt(solenoid_valve_timer, &onTimer_2, true);  // Attach interrupt
  timerAlarmWrite(Comp_on_off_timer, 1000000, true);             // Match value= 1000000 for 1 sec. delay.
  timerAlarmWrite(solenoid_valve_timer, 1000000, true);          // Match value= 1000000 for 1 sec. delay.
  wait_time = millis();
  update_values = millis();
}

// float temps[4] = [Comp1.SuctionTemp,Comp1.dischargeTemp,Comp1.SprayTemp,Comp1.SupplyTemp];


void loop() {

  // float temps = [Comp1.SuctionTemp,Comp1.dischargeTemp,Comp1.SprayTemp,Comp1.SupplyTemp];
  if (millis() - wait_time >= 1000) {
    // tempSensors.updateStatus(temps);
    // Comp1.SuctionTemp = temps[0];
    // Comp1.dischargeTemp = temps[1];
    // Comp1.SprayTemp = temps[2];
    // Comp1.SupplyTemp = temps[3];
    // Comp2.SuctionTemp = temps[4];
    // Comp2.dischargeTemp = temps[5];
#ifdef DEBUG
    // Serial.println("--------------------------------------------------");
    // Serial.print("Compressor 1 Suction Temperature: ");
    // Serial.println(Comp1.SuctionTemp);
    // Serial.println("--------------------------------------------------");
    // Serial.print("Compressor 1 Discharge Temperature: ");
    // Serial.println(Comp1.dischargeTemp);
    // Serial.println("--------------------------------------------------");
    // Serial.print("Compressor 1 Spray Temperature: ");
    // Serial.println(Comp1.SprayTemp);
    // Serial.println("--------------------------------------------------");
    // Serial.print("Compressor 1 Supply Temperature: ");
    // Serial.println(Comp1.SupplyTemp);
    // Serial.println("--------------------------------------------------");
    // Serial.print("Compressor 1 Supply Temperature: ");
    // Serial.println(Comp2.SuctionTemp);
    // Serial.println("--------------------------------------------------");
    // Serial.print("Compressor 1 Supply Temperature: ");
    // Serial.println(Comp2.dischargeTemp);
#endif
    wait_time = millis();
    save_in_holding_registers();

    // writeSingleRegister(1, 0, int16_t(holdingRegisters[0]));
    // writeSingleRegister(1, 1, int16_t(holdingRegisters[1]));
    // writeSingleRegister(1, 2, int16_t(holdingRegisters[2]));
    // writeSingleRegister(1, 3, int16_t(holdingRegisters[3]));

    // writeMultipleRegisters(1, 0, holdingRegisters, 50);
  }







  // delay(1000);

  // save_in_holding_registers();

  // if(millis()-update_values >= 1000){
  //   handleModbusRequest();
  //   update_values = millis();

  // }
  // handleModbusRequest();


  // bool all_safe = true;

  // readInputs(inputs);

  // all_safe = safety_check();


  // writeSingleRegister(MODBUS_SLAVE_ID, Control_Command, 0x01);  // For Forward
  // writeSingleRegister(MODBUS_SLAVE_ID, Control_Command, 0x05);  // For Stop


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /* Logic for turning on00000000.. Compressor and Solenoid Valve*/



  // -------------------Compressor On / Off Timer Start------------------
  // if (start) {
  //   if (safety_check()) {
  //     if (ra_temp >= r_a_temp_Setpoint) {
  //       if (compressor_onoff == 0) {
  //         compressor_timer_flag = true;
  //         timerAlarmEnable(Comp_on_off_timer);
  //       }
  //       // else if (compressor_onoff == 1 && solenoid_on_timer_flag == true) {
  //       //   totalInterruptCounter = 0;
  //       //   compressor_timer_flag = false;
  //       //   timerAlarmDisable(Comp_on_off_timer);
  //       //   timerWrite(Comp_on_off_timer, 0);
  //       // }
  //       else if (compressor_onoff == 1 && (solenoid_on_timer_flag == false && sv_l_u == 0)) {
  //         solenoid_on_timer_flag = true;
  //         timerAlarmEnable(solenoid_valve_timer);
  //       }
  //     } else {
  //       solenoid_on_timer_flag = false;
  //       sv_l_u = 0;
  //       if (compressor_onoff == 1) {
  //         compressor_timer_flag = true;
  //         timerAlarmEnable(Comp_on_off_timer);
  //       }
  //     }
  //   }
  // } else {
  //   solenoid_on_timer_flag = false;
  //   sv_l_u = 0;
  //   if (compressor_onoff == 1) {
  //     compressor_timer_flag = true;
  //     timerAlarmEnable(Comp_on_off_timer);
  //   } else {
  //     compressor_timer_flag = false;
  //     totalInterruptCounter = 0;
  //     compressor_timer_flag = false;
  //     timerAlarmDisable(Comp_on_off_timer);
  //     timerWrite(Comp_on_off_timer, 0);
  //   }
  // }

  // if (compressor_timer_flag == true) {
  //   if (interruptCounter > 0) {

  //     portENTER_CRITICAL(&timerMux);
  //     interruptCounter = 0;
  //     portEXIT_CRITICAL(&timerMux);

  //     totalInterruptCounter++;  //counting total interrupt

  //     if (compressor_onoff == 0) {
  //       Serial.print("Compressor is turning on after ");
  //       Serial.print(totalInterruptCounter);
  //       Serial.println(" seconds.");

  //       if (totalInterruptCounter > compressor_on_delay) {
  //         on_compressor();
  //         solenoid_on_timer_flag = true;
  //         timerAlarmEnable(solenoid_valve_timer);
  //         Serial.println("Solenoid Timer Flag On ---------------");
  //       }
  //     }       //
  //     else {  // If Compressor On
  //       Serial.print("Compressor is turning Off after ");
  //       Serial.print(totalInterruptCounter);
  //       Serial.println(" seconds because Solenoid Valve is Off.");

  //       if (totalInterruptCounter > compressor_off_delay) {
  //         off_compressor();
  //       }
  //     }
  //   }
  // }

  // if (Serial.available() > 0) {
  //   new_setpoint = Serial.readStringUntil('\n').toDouble();
  //   if (new_setpoint != ra_temp) {
  //     if (new_setpoint > 0) {
  //       ra_temp = new_setpoint;
  //       Serial.println("New ra_temp: ");
  //       Serial.println(ra_temp);
  //       delay(1000);
  //     }
  //   }
  // }

  // if (ra_temp >= r_a_temp_Setpoint) {
  //   if (compressor_onoff == 1 && sv_l_u == 0) {
  //     solenoid_on_timer_flag = true;
  //     timerAlarmEnable(solenoid_valve_timer);
  //   }  //
  //   else if (compressor_onoff == 1 && sv_l_u == 1) {
  //     solenoid_on_timer_flag = false;
  //     totalInterruptCounter_2 = 0;
  //     timerAlarmDisable(solenoid_valve_timer);
  //     timerWrite(solenoid_valve_timer, 0);
  //   }  //
  //   else if (compressor_onoff == 0 && sv_l_u == 1) {
  //     sv_l_u = 0;
  //     solenoid_on_timer_flag = false;
  //     totalInterruptCounter_2 = 0;
  //     timerAlarmDisable(solenoid_valve_timer);
  //     timerWrite(solenoid_valve_timer, 0);
  //   }  //
  // } else {
  //   sv_l_u = 0;
  //   solenoid_on_timer_flag = false;
  //   timerAlarmDisable(solenoid_valve_timer);
  // }

  // Serial.println(ra_temp);
  // Serial.println(r_a_temp_Setpoint);

  // if (solenoid_on_timer_flag == true) {
  //   Serial.println("Solenoid Timer Flag True");

  //   if (interruptCounter_2 > 0) {

  //     portENTER_CRITICAL(&timerMux);
  //     interruptCounter_2--;
  //     portEXIT_CRITICAL(&timerMux);

  //     totalInterruptCounter_2++;  //counting total interrupt

  //     Serial.print("Solenoid Valve is turning on after ");
  //     Serial.print(totalInterruptCounter_2);
  //     Serial.println(" seconds.");

  //     if (totalInterruptCounter_2 > solenoid_valve_on_delay) {
  //       on_solenoid();
  //     }
  //   }
  // } else {
  //   totalInterruptCounter_2 = 0;
  //   timerAlarmDisable(solenoid_valve_timer);
  //   timerWrite(solenoid_valve_timer, 0);
  //   // Serial.println("Solenoid Timer Flag False");
  // }



  // if (timer1_flag == true) {
  //   if (interruptCounter > 0) {

  //     portENTER_CRITICAL(&timerMux);
  //     interruptCounter--;
  //     portEXIT_CRITICAL(&timerMux);

  //     totalInterruptCounter++;  //counting total interrupt

  //     Serial.print("Compressor is turning Off after ");
  //     Serial.print(totalInterruptCounter);
  //     Serial.println(" seconds because Solenoid Valve is Off.");

  //     if (totalInterruptCounter > compressor_off_delay) {
  //       Serial.println("Counter Completed for Turning OFF Compressor");
  //       Serial.println("Turned OFF Compressor");
  //       compressor_onoff = 0;
  //       totalInterruptCounter = 0;
  //       timerAlarmDisable(Comp_on_off_timer);
  //       timer1_flag = false;
  //       timerWrite(Comp_on_off_timer, 0);
  //     }
  //   }
  // }
  /*---------------------------------------------------*/
  /*
  // write_on_EXV(0);
*/




  //-------------Code for EXV-----------------


  //////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////

  // EXV should be on 0 before start and should be 0 after at the time the system stops

  ///////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////



  // if (exv_state == reset) {
  if (currentStep < 50) {

    Serial.println("EXV_STATE_RESET, moving to 0");
    write_on_EXV(0);

    if (currentStep == 0) {

      Serial.println("EXV_STATE_RESET, moved to 0");
      exv_state = initializing;

      digitalWrite(EXV_1, HIGH);
      digitalWrite(EXV_2, HIGH);
      digitalWrite(EXV_3, HIGH);
      digitalWrite(EXV_4, HIGH);
      // exv_state = ready;
    }
  }  //
  else {

    Serial.println("EXV_STATE_RESET, moving to 125");
    write_on_EXV(125);

    if (currentStep == 125) {

      Serial.println("EXV_STATE_RESET, moved to 125");
      exv_state = initializing;

      digitalWrite(EXV_1, HIGH);
      digitalWrite(EXV_2, HIGH);
      digitalWrite(EXV_3, HIGH);
      digitalWrite(EXV_4, HIGH);
    }
  }
  // }  //
  // else if (exv_state == initializing) {

  //   // Serial.println("EXV_STATE_INITIALIZING, moving to 50");
  //   write_on_EXV(50);

  //   if (currentStep == 50) {
  //     exv_state = ready;
  //   }
  // }  //
  // else if (exv_state == ready) {

  //   // Serial.println("EXV_STATE_READY");
  //   write_on_EXV(int(Output));
  // }

  /*

#ifdef DEBUG
  Serial.print("Temperature By ds18b20(in Fahrenheit): ");
  Serial.println(sl_temp);
#endif

  // At 4 mA ADC = 5324 and At 20 mA ADC = 26701
  dl_pres = Measure_Pres(2, 5324, 26701, 0, 500);
  sl_pres = Measure_Pres(3, 5324, 26701, 0, 500);
  sl_temp_by_pres = calc_temp_from_pt_chart(sl_pres);
*/
  // if (compressor_onoff == 1) {
  //   if (dl_pres >= power_on_pres && condenser_fan_1 == 0) {
  //     condenser_fan_1 = 1;
  //   }  //
  //   else if (dl_pres <= cut_off_pres && condenser_fan_1 == 1) {
  //     condenser_fan_1 = 0;
  //   }
  // } else {
  //   if (condenser_fan_1 == 1) {
  //     condenser_fan_1 = 0;
  //   }
  // }
  /*
#ifdef DEBUG
  Serial.print("Discharge Line Pressure : ");
  Serial.println(dl_pres);
  Serial.print("Suction Line Pressure : ");
  Serial.println(sl_pres);
  Serial.print("Temperature by PT Chart(in Fahrenheit): ");
  Serial.println(sl_temp_by_pres);
#endif
*/

  // Suction_Super_Heat = sl_temp - sl_temp_by_pres;

  // #ifdef DEBUG
  // Serial.print("Suction_Super_Heat: ");
  // Serial.println(Suction_Super_Heat);
  // #endif

  // Input = Suction_Super_Heat;
  /*
  if (Serial.available() > 0) {
    new_setpoint = Serial.readStringUntil('\n').toDouble();
    if (new_setpoint != Setpoint) {
      if (new_setpoint > 0) {
        Setpoint = new_setpoint;
        Serial.println("New Setpoint: ");
        Serial.println(Setpoint);
        delay(1000);
      }
    }
  }
*/
  // myPID.Compute();
  /*
#ifdef DEBUG

  Serial.print("Output: ");
  Serial.println(Output);
  Serial.print("currentStep: ");
  Serial.println(currentStep);
  Serial.print("Counter For Preferences: ");
  Serial.println(counter);

  Serial.println(Output_Data_2[0]);
  Serial.println(Output_Data_2[1]);
  Serial.println(Output_Data_2[2]);
  Serial.println(Output_Data_2[3]);

  Serial.println("");

#endif

  /*----------------------------------------------*/

  // writeOutputs(Output_Data_2);





  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
