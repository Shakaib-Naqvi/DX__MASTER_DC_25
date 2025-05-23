// Define modbus parameters
// #define MODBUS_SLAVE_ID 0x01
// #define MODBUS_FUNCTION_READ_HOLDING_REGISTERS 0x03

// Read and Write Registers
#define Control_Command 0x1000 // 01H for Forward and 05H for Stop
#define Communication_Setting 0x2000

// Read Only Status Parameters Addresses
#define Output_Frequency 0x3000
#define Output_Voltage 0x3003
#define Output_Current 0x3004
#define Output_Power 0x3006

// #define MODBUS_REGISTER_COUNT 10
// #define MODBUS_BAUD_RATE 38400

// void writevfdSingleRegister(uint8_t slaveId, uint16_t address, uint16_t value) {
//   // Set the DE/RE pin high to enable RX mode
//   digitalWrite(RS485_DE_RE_PIN, HIGH);
//   uint8_t request[8];
//   request[0] = slaveId;
//   request[1] = 0x06;  // Function code for writing a single register
//   request[2] = address >> 8;
//   request[3] = address & 0xFF;
//   request[4] = value >> 8;
//   request[5] = value & 0xFF;
//   uint16_t crc = calculateCRC16(request, 6);
//   request[6] = crc & 0xFF;
//   request[7] = crc >> 8;
//   RS485Serial.write(request, sizeof(request));
//   RS485Serial.flush();  // Ensure all data is sent
//   delay(200);
//   // Set the DE/RE pin low to enable TX mode
//   digitalWrite(RS485_DE_RE_PIN, LOW);
// }



// // uint16_t calculateCRC16(uint8_t *data, uint16_t length) {
// //   uint16_t crc = 0xFFFF;
// //   for (uint16_t i = 0; i < length; i++) {
// //     crc ^= data[i];
// //     for (uint8_t j = 0; j < 8; j++) {
// //       if (crc & 0x0001) {
// //         crc >>= 1;
// //         crc ^= 0xA001;
// //       } else {
// //         crc >>= 1;
// //       }
// //     }
// //   }
// //   return crc;
// // }

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

// void writeSingleRegister(uint8_t slaveId, uint16_t address, uint16_t value) {
//   // Set the DE/RE pin high to enable RX mode
//   digitalWrite(RS485_DE_RE_PIN, LOW);
//   uint8_t request[8];
//   request[0] = slaveId;
//   request[1] = 0x06;  // Function code for writing a single register
//   request[2] = address >> 8;
//   request[3] = address & 0xFF;
//   request[4] = value >> 8;
//   request[5] = value & 0xFF;
//   uint16_t crc = calculateCRC16(request, 6);
//   request[6] = crc & 0xFF;
//   request[7] = crc >> 8;
//   RS485Serial.write(request, sizeof(request));

//   // Set the DE/RE pin low to enable TX mode
//   digitalWrite(RS485_DE_RE_PIN, HIGH);
// }


// void sendModbusRequest(uint8_t slaveId, uint8_t functionCode, uint16_t startAddress, uint16_t registerCount) {
//   uint8_t request[8];
//   request[0] = slaveId;
//   request[1] = functionCode;
//   request[2] = startAddress >> 8;
//   request[3] = startAddress & 0xFF;
//   request[4] = registerCount >> 8;
//   request[5] = registerCount & 0xFF;
//   uint16_t crc = calculateCRC16(request, 6);
//   request[6] = crc & 0xFF;
//   request[7] = crc >> 8;
//   RS485Serial.write(request, sizeof(request));
// }

// bool readModbusResponse(uint8_t *response, uint16_t length) {
//   uint32_t timeout = millis() + 1500;
//   uint16_t index = 0;
//   while (millis() < timeout) {
//     if (RS485Serial.available()) {
//       response[index++] = RS485Serial.read();
//       if (index >= length) {
//         return true;
//       }
//     }
//   }
//   return false;
// }

// bool readHoldingRegisters(uint8_t slaveId, uint16_t startAddress, uint16_t registerCount, uint16_t *buffer) {
//   digitalWrite(RS485_DE_RE_PIN, HIGH);  // Set DE/RE pin high for transmit mode
//   sendModbusRequest(slaveId, MODBUS_FUNCTION_READ_HOLDING_REGISTERS, startAddress, registerCount);
//   RS485Serial.flush();                 // Ensure all data is sent
//   digitalWrite(RS485_DE_RE_PIN, LOW);  // Set DE/RE pin low for receive mode
//   uint8_t response[5 + 2 * registerCount];
//   delay(250);
//   if (readModbusResponse(response, sizeof(response))) {
//     uint16_t crc = (response[sizeof(response) - 1] << 8) | response[sizeof(response) - 2];
//     if (calculateCRC16(response, sizeof(response) - 2) == crc) {
//       for (int i = 0; i < registerCount; i++) {
//         buffer[i] = (response[3 + i * 2] << 8) | response[4 + i * 2];
//       }
//       return true;
//     } else {
// #ifdef DEBUG
//       Serial.println("CRC error!");
// #endif
//     }
//   } else {
// #ifdef DEBUG
//     Serial.println("No response or timeout!");
// #endif
//   }
//   return false;
// }


// int read_beca(uint8_t register_value) {
//   if (readHoldingRegisters(MODBUS_SLAVE_ID, MODBUS_REGISTER_ADDRESS, MODBUS_REGISTER_COUNT, registers)) {
//     for (int i = 0; i < MODBUS_REGISTER_COUNT; i++) {
// #ifdef DEBUG
//       Serial.print("Register ");
//       Serial.print(MODBUS_REGISTER_ADDRESS + i);
//       Serial.print(": ");
//       Serial.println(registers[i]);
// #endif
//     }
//   } else {
// #ifdef DEBUG
//     Serial.println("Failed to read registers");
// #endif
//   }
//   int Temp_BECA = registers[register_value];
//   return Temp_BECA;
// }

