// // #define MODBUS_BAUD_RATE 115200
// #define MODBUS_BAUD_RATE 9600
// #define RS485_RX_PIN 16
// #define RS485_TX_PIN 17
// #define RS485_DE_RE_PIN 4  // DE/RE pin for controlling TX/RX mode
// #define MODBUS_SLAVE_ID 1
// #define MODBUS_FUNCTION_READ_HOLDING_REGISTERS 0x03
// #define MODBUS_FUNCTION_WRITE_SINGLE_REGISTER 0x06

// HardwareSerial RS485Serial(1);  // Using Serial1 for RS485 communication
uint16_t holdingRegisters[50];  // Holding register example
// void readHoldingRegisters(uint16_t registerAddress, uint16_t numRegisters);
// void writeSingleRegister(uint16_t registerAddress, uint16_t value);
// void sendErrorResponse(uint8_t slaveId, uint8_t functionCode, uint8_t exceptionCode);
// void sendModbusResponse(byte* response, size_t length);
// void writeFloatToRegisters(uint16_t registerAddress, float value);

// uint16_t calculateCRC16(uint8_t* data, uint16_t length) {

//   uint16_t crc = 0xFFFF;
//   for (uint16_t i = 0; i < length; i++) {
//     crc ^= (uint16_t)data[i];
//     for (uint8_t j = 0; j < 8; j++) {
//       if (crc & 0x0001) {
//         crc >>= 1;
//         crc ^= 0xA001;
//       } else {
//         crc >>= 1;
//       }
//     }
//   }
//   return crc;
// }


// byte request[8];
// uint8_t slaveId, functionCode;
// uint16_t registerAddress, valueOrLength;
// void handleModbusRequest() {

//   digitalWrite(RS485_DE_RE_PIN, LOW);  // Set DE/RE pin to receive mode
//   uint8_t request[16];                 // Adjust based on expected max frame size
//   int8_t len = 0;

//   unsigned long startMillis = millis();  // Timeout for waiting
//   while (millis() - startMillis < 50) {  // Adjust timeout if needed
//     if (RS485Serial.available()) {
//       request[len++] = RS485Serial.read();
//       startMillis = millis();  // Reset timeout on receiving data
//     }
//     if (len >= 8) break;  // Assume minimum Modbus frame length
//   }

//   if (len < 8) {
//     // Serial.println("Incomplete Modbus request, ignoring.");
//     return;
//   }

//   Serial.print("Received request: ");
//   for (int i = 0; i < len; i++) {
//     Serial.print(request[i], HEX);
//     Serial.print(" ");
//   }
//   Serial.println();

//   uint8_t slaveId = request[0];
//   if (slaveId != MODBUS_SLAVE_ID) {
//     Serial.print("Ignoring request for slave ID: ");
//     Serial.println(slaveId);
//     while (RS485Serial.available()) RS485Serial.read();  // Flush buffer
//     return;
//   }



//   slaveId = request[0];
//   functionCode = request[1];
//   registerAddress = (request[2] << 8) | request[3];
//   valueOrLength = (request[4] << 8) | request[5];
//   if (slaveId != MODBUS_SLAVE_ID) {
//     Serial.print("Ignoring request for slave ID: ");
//     Serial.println(slaveId);
//     RS485Serial.flush();
//     delay(5);
//     return;
//   }

//   // CRC Check

//   uint16_t receivedCRC = (request[6] | (request[7] << 8));
//   uint16_t calculatedCRC = calculateCRC16(request, 6);

//   if (receivedCRC != calculatedCRC) {
// #ifdef DEBUG
//     Serial.println("CRC check failed!");
// #endif
//     crc_error_count++;
//     if (crc_error_count >= 10) {
// #ifdef DEBUG
//       Serial.println("CRC errors exceeded limit! Reinitializing RS-485...");
// #endif
//       RS485Serial.end();
//       delay(20);

//       RS485Serial.begin(MODBUS_BAUD_RATE, SERIAL_8E1, RS485_RX_PIN, RS485_TX_PIN);
//       delay(20);
//       crc_error_count = 0;
//     }
//     return;
//   }

//   if (functionCode == MODBUS_FUNCTION_READ_HOLDING_REGISTERS) {
//     readHoldingRegisters(registerAddress, valueOrLength);
//   }  //
//   else if (functionCode == MODBUS_FUNCTION_WRITE_SINGLE_REGISTER) {
//     writeSingleRegister(registerAddress, valueOrLength);
//     readHoldingRegisters(registerAddress, valueOrLength);
//   }  //
//   else if (functionCode == 0x01) {
//     writeSingleRegister(registerAddress, valueOrLength);
//     readHoldingRegisters(registerAddress, valueOrLength);
//     delay(10);
//   } else {
//     sendErrorResponse(slaveId, functionCode, 0x01);  // 0x01 = Illegal function
//   }
// }




// void readHoldingRegisters(uint16_t registerAddress, uint16_t numRegisters) {
//   if (registerAddress + numRegisters > 100) {  // Check register range
// #ifdef DEBUG
//     Serial.println("Invalid register range.");
// #endif

//     return;
//   }

//   uint8_t response[5 + 2 * numRegisters];
//   response[0] = MODBUS_SLAVE_ID;
//   response[1] = MODBUS_FUNCTION_READ_HOLDING_REGISTERS;
//   response[2] = numRegisters * 2;  // Byte count

//   for (int i = 0; i < numRegisters; i++) {
//     response[3 + i * 2] = (holdingRegisters[registerAddress + i] >> 8) & 0xFF;  // High byte
//     response[4 + i * 2] = holdingRegisters[registerAddress + i] & 0xFF;         // Low byte
//   }

//   uint16_t crc = calculateCRC16(response, 3 + 2 * numRegisters);
//   response[3 + 2 * numRegisters] = crc & 0xFF;
//   response[4 + 2 * numRegisters] = (crc >> 8) & 0xFF;

//   sendModbusResponse(response, 5 + 2 * numRegisters);
// #ifdef DEBUG
//   for (int i = 0; i < (5 + 2 * numRegisters); i++) {
//     Serial.print(response[i], HEX);
//     Serial.print(" ");
//   }
//   Serial.println("");
// #endif
// }

// void writeSingleRegister(uint16_t registerAddress, uint16_t value) {
//   if (registerAddress >= 100) {  // Check if address is within range

// #ifdef DEBUG
//     Serial.println("Invalid register address.");
// #endif

//     return;
//   }

//   if (registerAddress == 11) {
//     Setpoint = value;
//   }
//   holdingRegisters[registerAddress] = value;  // High byte second

//   preferences.begin("hmi_values", false);

//   switch (registerAddress) {
//     case 11:
//       r_a_temp_Setpoint = value;
//       preferences.putFloat("r_a_t_sp", r_a_temp_Setpoint);
//       Serial.println("r_a_temp_Setpoint saved");
//       break;

//     case 25:
//       Low_Temperature_Alarm = value;
//       preferences.putFloat("low_t_al", Low_Temperature_Alarm);
//       Serial.println("Low_Temperature_Alarm saved");
//       break;

//     case 26:
//       High_Temperature_Alarm = value;
//       preferences.putFloat("high_t_al", High_Temperature_Alarm);
//       Serial.println("High_Temperature_Alarm saved");
//       break;

//     case 27:
//       Spray_Temperature_Alarm = value;
//       preferences.putFloat("spr_t_al", Spray_Temperature_Alarm);
//       Serial.println("Spray_Temperature_Alarm saved");
//       break;

//     case 34:
//       start = value;
//       // if (value == 1) {
//       //   defrosting_timer_en = true;
//       // }
//       preferences.putBool("start", start);  // Key name already short
//       break;

//     case 36:
//       compressor_on_delay = value;
//       preferences.putUInt("comp_on", compressor_on_delay);
//       Serial.println("compressor_on_delay saved");
//       break;

//     case 37:
//       gas_selected_s = 0;
//       if (value == 1) {
//         holdingRegisters[38] = 0;
//         holdingRegisters[39] = 0;
//         holdingRegisters[40] = 0;
//         holdingRegisters[41] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       preferences.putUInt("gas_sel_s", gas_selected_s);
//       Serial.println("gas_selected_s saved");
//       break;

//     case 38:
//       gas_selected_s = 1;
//       if (value == 1) {
//         holdingRegisters[37] = 0;
//         holdingRegisters[39] = 0;
//         holdingRegisters[40] = 0;
//         holdingRegisters[41] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       preferences.putUInt("gas_sel_s", gas_selected_s);
//       Serial.println("gas_selected_s saved");
//       break;

//     case 39:
//       gas_selected_s = 2;
//       if (value == 1) {
//         holdingRegisters[37] = 0;
//         holdingRegisters[38] = 0;
//         holdingRegisters[40] = 0;
//         holdingRegisters[41] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       preferences.putUInt("gas_sel_s", gas_selected_s);
//       Serial.println("gas_selected_s saved");
//       break;

//     case 40:
//       gas_selected_s = 3;
//       if (value == 1) {
//         holdingRegisters[37] = 0;
//         holdingRegisters[38] = 0;
//         holdingRegisters[39] = 0;
//         holdingRegisters[41] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       preferences.putUInt("gas_sel_s", gas_selected_s);
//       Serial.println("gas_selected_s saved");
//       break;

//     case 41:
//       gas_selected_s = 4;
//       if (value == 1) {
//         holdingRegisters[37] = 0;
//         holdingRegisters[38] = 0;
//         holdingRegisters[39] = 0;
//         holdingRegisters[40] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       preferences.putUInt("gas_sel_s", gas_selected_s);
//       Serial.println("gas_selected_s saved");
//       break;

//     case 42:
//       if (value == 1) {
//         holdingRegisters[43] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       break;

//     case 43:
//       if (value == 1) {
//         holdingRegisters[42] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       break;

//     case 44:
//       if (value == 1) {
//         holdingRegisters[45] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       break;

//     case 45:
//       if (value == 1) {
//         holdingRegisters[44] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       break;

//       // case 46:
//       //   defrosting_on_delay = value;
//       //   preferences.putUInt("def_on", defrosting_on_delay);
//       //   Serial.println("defrosting_on_delay saved");
//       //   break;

//       // case 47:
//       //   defrosting_off_delay = value;
//       //   preferences.putUInt("def_off", defrosting_off_delay);
//       //   Serial.println("defrosting_off_delay saved");
//       //   break;

//     case 52:
//       gas_selected_d = 5;
//       if (value == 1) {
//         holdingRegisters[53] = 0;
//         holdingRegisters[54] = 0;
//         holdingRegisters[55] = 0;
//         holdingRegisters[56] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       preferences.putUInt("gas_sel_d", gas_selected_d);
//       Serial.println("gas_selected_d saved");
//       break;

//     case 53:
//       gas_selected_d = 6;
//       if (value == 1) {
//         holdingRegisters[52] = 0;
//         holdingRegisters[54] = 0;
//         holdingRegisters[55] = 0;
//         holdingRegisters[56] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       preferences.putUInt("gas_sel_d", gas_selected_d);
//       Serial.println("gas_selected_d saved");
//       break;

//     case 54:
//       gas_selected_d = 7;
//       if (value == 1) {
//         holdingRegisters[52] = 0;
//         holdingRegisters[53] = 0;
//         holdingRegisters[55] = 0;
//         holdingRegisters[56] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       preferences.putUInt("gas_sel_d", gas_selected_d);
//       Serial.println("gas_selected_d saved");
//       break;

//     case 55:
//       gas_selected_d = 8;
//       if (value == 1) {
//         holdingRegisters[52] = 0;
//         holdingRegisters[53] = 0;
//         holdingRegisters[54] = 0;
//         holdingRegisters[56] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       preferences.putUInt("gas_sel_d", gas_selected_d);
//       Serial.println("gas_selected_d saved");
//       break;

//     case 56:
//       gas_selected_d = 9;
//       if (value == 1) {
//         holdingRegisters[52] = 0;
//         holdingRegisters[53] = 0;
//         holdingRegisters[54] = 0;
//         holdingRegisters[55] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       preferences.putUInt("gas_sel_d", gas_selected_d);
//       Serial.println("gas_selected_d saved");
//       break;

//     case 57:
//       if (value == 1) {
//         holdingRegisters[58] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       break;

//     case 58:
//       if (value == 1) {
//         holdingRegisters[57] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       break;

//     case 59:
//       if (value == 1) {
//         holdingRegisters[60] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       break;

//     case 60:
//       if (value == 1) {
//         holdingRegisters[59] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       break;

//       // case 61:
//       //   if (value == 1) {
//       //     defrosting_en = true;
//       //     defrosting_timer_en = true;
//       //     holdingRegisters[62] = 0;
//       //   } else {
//       //     holdingRegisters[registerAddress] = 1;
//       //   }
//       //   break;

//       // case 62:
//       //   if (value == 1) {
//       //     defrosting_en = false;
//       //     holdingRegisters[61] = 0;
//       //   } else {
//       //     holdingRegisters[registerAddress] = 1;
//       //   }
//       //   break;

//       // case 65:
//       //   defrosting_sw_timer = true;
//       //   defrosting_sw_temp = false;
//       //   if (value == 1) {
//       //     holdingRegisters[66] = 0;
//       //   } else {
//       //     holdingRegisters[registerAddress] = 1;
//       //   }
//       //   preferences.putBool("def_sw_t", defrosting_sw_timer);
//       //   preferences.putBool("def_sw_T", defrosting_sw_temp);
//       //   Serial.println("defrosting_sw_timer and defrosting_sw_temp saved");
//       //   break;

//       // case 66:
//       //   defrosting_sw_timer = false;
//       //   defrosting_sw_temp = true;
//       //   if (value == 1) {
//       //     holdingRegisters[65] = 0;
//       //   } else {
//       //     holdingRegisters[registerAddress] = 1;
//       //   }
//       //   preferences.putBool("def_sw_t", defrosting_sw_timer);
//       //   preferences.putBool("def_sw_T", defrosting_sw_temp);
//       //   Serial.println("defrosting_sw_timer and defrosting_sw_temp saved");
//       //   break;

//       // case 67:
//       //   tolerance = value;
//       //   preferences.putInt("tol", tolerance);
//       //   Serial.println("tolerance saved");
//       //   break;

//       // case 68:
//       //   Heat_Setpoint = value;
//       //   preferences.putInt("heat_sp", Heat_Setpoint);
//       //   Serial.println("Heat_Setpoint saved");
//       //   break;

//     case 69:
//       compressor_off_delay = value;
//       preferences.putUInt("comp_off", compressor_off_delay);
//       Serial.println("compressor_off_delay saved");
//       break;

//     case 71:
//       power_on_pres = value;
//       preferences.putInt("pwr_on", power_on_pres);
//       Serial.println("power_on_pres saved");
//       break;

//     case 72:
//       cut_off_pres = value;
//       preferences.putInt("cut_off", cut_off_pres);
//       Serial.println("cut_off_pres saved");
//       break;

//     case 73:
//       solenoid_valve_on_delay = value;
//       preferences.putUInt("sol_on", solenoid_valve_on_delay);
//       Serial.println("solenoid_valve_on_delay saved");
//       break;

//       // case 74:
//       //   defrosting_electrical = true;
//       //   defrosting_mechanical = false;
//       //   if (value == 1) {
//       //     holdingRegisters[75] = 0;
//       //   } else {
//       //     holdingRegisters[registerAddress] = 1;
//       //   }
//       //   preferences.putBool("def_elec", defrosting_electrical);
//       //   preferences.putBool("def_mech", defrosting_mechanical);
//       //   Serial.println("defrosting_electrical and defrosting_mechanical saved");
//       //   break;

//       // case 75:
//       //   defrosting_electrical = false;
//       //   defrosting_mechanical = true;
//       //   if (value == 1) {
//       //     holdingRegisters[74] = 0;
//       //   } else {
//       //     holdingRegisters[registerAddress] = 1;
//       //   }
//       //   preferences.putBool("def_elec", defrosting_electrical);
//       //   preferences.putBool("def_mech", defrosting_mechanical);
//       //   Serial.println("defrosting_electrical and defrosting_mechanical saved");
//       //   break;

//     case 76:
//       condenser_fan_enable = true;
//       condenser_fan_disable = false;
//       if (value == 1) {
//         holdingRegisters[77] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       preferences.putBool("cond_en", condenser_fan_enable);
//       preferences.putBool("cond_dis", condenser_fan_disable);
//       Serial.println("condenser_fan_enable and condenser_fan_disable saved");
//       break;

//     case 77:
//       condenser_fan_enable = false;
//       condenser_fan_disable = true;
//       if (value == 1) {
//         holdingRegisters[76] = 0;
//       } else {
//         holdingRegisters[registerAddress] = 1;
//       }
//       preferences.putBool("cond_en", condenser_fan_enable);
//       preferences.putBool("cond_dis", condenser_fan_disable);
//       Serial.println("condenser_fan_enable and condenser_fan_disable saved");
//       break;

//     case 90:
//       vfd_start = value;
//       // if (value == 1) {
//       //   defrosting_timer_en = true;
//       // }
//       if (vfd_start == 1) {
//         // writevfdSingleRegister(MODBUS_SLAVE_ID, 0x1000, 1);  // Command to run
//       } else {
//         // writevfdSingleRegister(MODBUS_SLAVE_ID, 0x1000, 5);  // Command to stop
//       }
//       preferences.putBool("vfd_start", vfd_start);  // Key name already short
//       break;


//     default:
//       break;
//   }

//   preferences.end();

// #ifdef DEBUG
//   Serial.print("Writing Value ");
//   Serial.print(value);
//   Serial.print(" to register address ");
//   Serial.println(registerAddress);
// #endif
// }


// void sendErrorResponse(uint8_t slaveId, uint8_t functionCode, uint8_t exceptionCode) {
//   uint8_t response[5] = {
//     slaveId,
//     functionCode | 0x80,  // Set MSB to 1 for error
//     exceptionCode
//   };

//   uint16_t crc = calculateCRC16(response, 3);
//   response[3] = crc & 0xFF;
//   response[4] = (crc >> 8) & 0xFF;

//   sendModbusResponse(response, 5);
// }

// void sendModbusResponse(byte* response, size_t length) {
//   digitalWrite(RS485_DE_RE_PIN, HIGH);  // Enable TX mode
//   RS485Serial.write(response, length);
//   RS485Serial.flush();
//   delay(2);
//   digitalWrite(RS485_DE_RE_PIN, LOW);
// }
