/*
  DC25 based Chiller Master 
  -> One DC25 for each Circuit/Compressor
  -> Single HMI for both circuits via Ethernet Switch (Modbus TCP)
*/

#include "Def.h"
#include <Preferences.h>
#include "ModbusRTU.h"
#include "ADS1115.h"
// #include "Temp.h"
#include <QuickPID.h>
Preferences preferences;

TaskHandle_t Task1 = NULL;
QuickPID myPID(&Input, &Output, &Setpoint, kpp, kii, kdd, myPID.Action::reverse);

// *******************************************************************************************************************************************
void setup() {
  Serial.begin(115200);
  Serial.println("Serial OK.");

  RS485Serial.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  Serial.print("Loop Task Running on core : ");
  Serial.println(xPortGetCoreID());  // Core 1
  // scanSensors();
  // writeSingleRegister(VFD_1_ID, Control_Address, VFD_Stop);

  // preferences.begin("Setpoints", true);  // The true argument means that we’ll use it in read mode
  // // read_memory();
  // preferences.end();
  // ADSinit();
  // EXV_CONFIG();

  // EXV.currentStep = 550;
  // while (EXV.currentStep > 0) {
  //   // Close(InitialStepDelay);
  //   EXV.currentStep--;
  // }

  // Setpoint = 12.0;
  // Setpoint = (float)CompX.SuperHeatSV;
  // //apply PID gains
  // myPID.SetTunings(2, 0.5, 0);
  // myPID.SetOutputLimits(0, 100);  // (minEXVSteps, maxEXVSteps)
  // myPID.SetSampleTimeUs(20000);   // 20 ms
  // //turn the PID on
  // myPID.SetMode(myPID.Control::automatic);
}
float value = 0;

void loop() {
  writeSingleRegister(0x01, 0x00, value);
  writeSingleRegister(1, 0x00, value);
  writeSingleRegister(1, 0x01, value);
  writeSingleRegister(1, 0x02, value);
  writeSingleRegister(1, 0x03, value);
  writeSingleRegister(1, 0x04, value);
  value += 0.5;
  delay(2000);
  // scanSensors();
  // delay(1000);
}


// 01 06 00 00 00 0A 09 CD


// ******************************************************************************************************************************************
void scanSensors() {  // scans VFD Parameters , ADAM-4015 Temp module and Pressure ADCs and compares the vales within set ranges
  // scanADC();          // calculates Discharge and Suction Pressures
  // CompX.suctionPressure = suction + 2;
  // CompX.dischargePressure = discharge + 5;

  uint16_t tempReg[6] = { 0, 0, 0, 0, 0, 0 }, VFD_Reg[5] = { 0, 0, 0, 0, 0 };  // from ADAM-4015 module
  // if (readHoldingRegisters(VFD_1_ID, 0x3000, 5, VFD_Reg)) {
  //   CompX.out_freq = VFD_Reg[0];     //40011 - OUTPUT FREQ
  //   CompX.out_voltage = VFD_Reg[3];  //40012 - OUTPUT VOLTAGE
  //   CompX.out_amps = VFD_Reg[4];     //40013 - OUTPUT AMPS
  //   CompX.bus_voltage = VFD_Reg[2];  //40016 - BUS VOLTAGE
  // } else {
  //   Serial.println("VFD # 1 Communication Error!");
  // }

  if (readHoldingRegisters(8, 0, 6, tempReg)) {
    for (int i = 0; i < 6; i++) {
      Serial.print(i);
      Serial.print(": ");
      Serial.println(tempReg[i]);
      if (tempReg[i] != 65535) {
        tempReg[i] = tempReg[i] / 1000;
        Serial.println(tempReg[i]);
      }
    }
    Serial.println();
  } else {
    Serial.println("Adam-4015 # 1 Communication Error!");
  }

  // uint16_t Sat_Temp = Temp_from_PT_chart(CompX.suctionPressure);  // Saturation Temp from PT chart
  // CompX.SuperHeatPV = CompX.SuctionTemp - Sat_Temp;               // if Superheat == actual suction temp that means suction pressure sensor is disconnected
}

void runVFD(uint16_t freq, uint16_t command) {
  switch (command) {
    case 0:  // stop operation
      // Output_Write(CompMotor, 0);
      writeSingleRegister(VFD_1_ID, Control_Address, VFD_Stop);
      break;
    case 1:  // run oepration
      // Output_Write(Condenser_Fan1, 1);
      writeSingleRegister(VFD_1_ID, Freq_Address, freq);  // Set frequency to 50.00 Hz (50.00 * 200 = 10000)
      writeSingleRegister(VFD_1_ID, Control_Address, VFD_Run);
      // Output_Write(CompMotor, 1);
      break;
    default:
      Serial.println("Invalid VFD Command !");
      break;
  }
}
