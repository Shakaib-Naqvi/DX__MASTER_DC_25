/*
    MODBUS RTU - AS A Master DEVICE USING RS485 to communicate with Adam4015 & INVT VFDs

    Function Code 0x03 – Read Holding Registers
    Function Code 0x06 – Write Single Register
*/

#include <Arduino.h>

// Define Modbus RS485 parameters
#define RTD_SLAVE_ID_1 3  // 1st ADAM 4015 ID
#define RTD_SLAVE_ID_2 8  // 2nd ADAM 4015 ID using this
#define VFD_1_ID 1
#define VFD_2_ID 2

#define RS485_BAUD_RATE 9600
#define RS485_TX_PIN 17
#define RS485_RX_PIN 16
#define RS485_DE_RE_PIN 4  // DE and RE pins tied together and connected to GPIO 4

#define Acceleration_Time_Address 0x0B
#define Deceleration_Time_Address 0x0C
#define Control_Address 0x1000
#define Freq_Address 0x2000  // Set Hz
#define VFD_Run 1
#define VFD_Stop 5

// ADAM - 4015 CH NO.
#define SUCTION_C1 2
#define DISCHARGE_C1 3
#define SPRAY_C1 0
#define SUB_COOLING_C1 1
#define CHW_IN 4
#define CHW_OUT 5

HardwareSerial RS485Serial(1);

bool readHoldingRegisters(uint8_t slaveId, uint16_t startAddress, uint16_t registerCount, uint16_t *buffer);
void writeSingleRegister(uint8_t slaveId, uint16_t address, uint16_t value);

uint16_t calculateCRC(uint8_t *buffer, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= buffer[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

void clearSerialBuffer() {
  while (RS485Serial.available()) RS485Serial.read();  // Clear any old data
}

void sendModbusRequest(uint8_t slaveId, uint8_t functionCode, uint16_t startAddress, uint16_t registerCount) {
  digitalWrite(RS485_DE_RE_PIN, HIGH);  // Set DE/RE pin high for transmit mode
  uint8_t request[8];
  request[0] = slaveId;
  request[1] = functionCode;
  request[2] = startAddress >> 8;
  request[3] = startAddress & 0xFF;
  request[4] = registerCount >> 8;
  request[5] = registerCount & 0xFF;
  uint16_t crc = calculateCRC(request, 6);
  request[6] = crc & 0xFF;
  request[7] = crc >> 8;

  RS485Serial.write(request, sizeof(request));
  RS485Serial.flush();  // Ensure all data is sent
  clearSerialBuffer();  // Ensure buffer is empty
  Serial.println("Sent Request: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(request[i]);
    Serial.print(" ");
  }
  Serial.println();
  digitalWrite(RS485_DE_RE_PIN, LOW);  // Set DE/RE pin low for receive mode
}

bool readModbusResponse(uint8_t *response, uint16_t length) {
  uint32_t startTime = millis();
  uint16_t index = 0;
  while (millis() - startTime < 500) {  // 1000ms timeout
    if (RS485Serial.available()) {
      response[index++] = RS485Serial.read();
      if (index >= length) return true;
    }
  }
  // Serial.println("Timeout or Incomplete Response!");
  return false;
}

bool readHoldingRegisters(uint8_t slaveId, uint16_t startAddress, uint16_t registerCount, uint16_t *buffer) {
  sendModbusRequest(slaveId, 0x03, startAddress, registerCount);
  delay(50);
  uint8_t response[5 + 2 * registerCount];
  if (readModbusResponse(response, sizeof(response))) {
    uint16_t crc = (response[sizeof(response) - 1] << 8) | response[sizeof(response) - 2];
    if (calculateCRC(response, sizeof(response) - 2) == crc) {
      for (int i = 0; i < registerCount; i++) {
        buffer[i] = (response[3 + i * 2] << 8) | response[4 + i * 2];
      }
      return true;
    } else {
      // Serial.println("CRC Error!");
    }
  } else {
    // Serial.println("Modbus RTU : No response or timeout!");upload kro
  }
  return false;
}
void writeSingleRegister(uint8_t slaveId, uint16_t address, uint16_t value) {
  sendModbusRequest(slaveId, 0x06, address, value);
  delay(500);
  // readHoldingRegisters(slaveId, address, 10, registers);
}





void sendModbusRequestt(uint8_t slaveId, uint8_t functionCode, uint16_t startAddress, uint16_t registerCount) {
  uint8_t request[8];
  request[0] = slaveId;
  request[1] = functionCode;
  request[2] = startAddress >> 8;
  request[3] = startAddress & 0xFF;
  request[4] = registerCount >> 8;
  request[5] = registerCount & 0xFF;
  uint16_t crc = calculateCRC(request, 6);
  request[6] = crc & 0xFF;
  request[7] = crc >> 8;
  RS485Serial.write(request, sizeof(request));
}

bool readModbusResponsee(uint8_t *response, uint16_t length) {
  uint32_t timeout = millis() + 1500;
  uint16_t index = 0;
  while (millis() < timeout) {
    if (RS485Serial.available()) {
      response[index++] = RS485Serial.read();
      if (index >= length) {
        return true;
      }
    }
  }
  return false;
}

bool readHoldingRegisterss(uint8_t slaveId, uint16_t startAddress, uint16_t registerCount, uint16_t *buffer) {
  // digitalWrite(RS485_DE_RE_PIN, HIGH);  // Set DE/RE pin high for transmit mode
  sendModbusRequestt(slaveId, 0x03, startAddress, registerCount);
  RS485Serial.flush();  // Ensure all data is sent
  // digitalWrite(RS485_DE_RE_PIN, LOW);  // Set DE/RE pin low for receive mode
  uint8_t response[5 + 2 * registerCount];
  delay(100);
  if (readModbusResponsee(response, sizeof(response))) {
    uint16_t crc = (response[sizeof(response) - 1] << 8) | response[sizeof(response) - 2];
    if (calculateCRC(response, sizeof(response) - 2) == crc) {
      for (int i = 0; i < registerCount; i++) {
        buffer[i] = (response[3 + i * 2] << 8) | response[4 + i * 2];
      }
      return true;
    } else {
// #ifdef DEBUG
      Serial.println("CRC error!");
// #endif
      RS485Serial.end();
      delay(50);
      RS485Serial.begin(RS485_BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
      delay(50);
    }
  } else {
// #ifdef DEBUG
    Serial.println("No response or timeout!");
// #endif
  }
  return false;
}

void writeSingleRegisterr(uint8_t slaveId, uint16_t address, uint16_t value) {
  // Set the DE/RE pin high to enable RX mode
  // digitalWrite(RS485_DE_RE_PIN, LOW);
  uint8_t request[8];
  request[0] = slaveId;
  request[1] = 0x06;  // Function code for writing a single register
  request[2] = address >> 8;
  request[3] = address & 0xFF;
  request[4] = value >> 8;
  request[5] = value & 0xFF;
  uint16_t crc = calculateCRC(request, 6);
  request[6] = crc & 0xFF;
  request[7] = crc >> 8;
  RS485Serial.write(request, sizeof(request));

  // Set the DE/RE pin low to enable TX mode
  // digitalWrite(RS485_DE_RE_PIN, HIGH);

  RS485Serial.flush();


#ifdef DEBUG
  Serial.print("Sent request: ");
  for (int i = 0; i < sizeof(request); i++) {
    Serial.print(request[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
#endif
  readHoldingRegisterss(slaveId, 0x00, 10, registers);
}
