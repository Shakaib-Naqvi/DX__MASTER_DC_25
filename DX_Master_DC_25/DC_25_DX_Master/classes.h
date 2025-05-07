#define ONE_WIRE_BUS 4   // DS18B20 pin
#define PRESSURE_PIN A0  // Analog pressure input
#define EXV_PIN 5        // PWM control pin for EXV (mockup)

// ========== Base Component Class ==========
class Component {
public:
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void checkStatus() = 0;
  virtual void updateStatus(int16_t *temp) {}
};

// ========== Temperature Sensor ==========
class TemperatureSensor : public Component {
private:
  float temperature;
  uint16_t tempReg[6];
public:
  TemperatureSensor()
    : temperature(0.0) {}
  void start() override {
    // sensors.begin();
  }
  void stop() override {}
  void checkStatus() override {}
  void updateStatus(int16_t *temp) override {

    if (readHoldingRegisters(RTD_SLAVE_ID_2, 0, 6, tempReg)) {
      for (int i = 0; i < 6; i++) {
        Serial.print(i);
        Serial.print(": ");
        Serial.println(tempReg[i]);
        if (tempReg[i] != 65535) {
          tempReg[i] = tempReg[i] / 1000;
          temp[i] = tempReg[i];
          Serial.println(temp[i]);
        } else {
          temp[i] = -888.8;
          Serial.println(temp[i]);
        }
      }
      Serial.println();
    } else {
      Serial.println("Adam-4015 # 1 Communication Error!");
    }
  }
  float getTemperature() {
    return temperature;
  }
};


// ========== Pressure Sensor ==========
// class PressureSensor : public Component {
// private:
//   float pressure;
// public:
//   PressureSensor()
//     : pressure(0.0) {}
//   void start() override {}
//   void stop() override {}
//   void checkStatus(int channel) override {
//     scanADC(channel);
//     // int raw = analogRead(PRESSURE_PIN);
//     pressure = map(raw, 0, 1023, 0, 150);  // scale example (0–150 psi)
//   }
//   float getPressure() {
//     return pressure;
//   }
// };

// ========== Compressor ==========
class Compressor : public Component {
private:
  bool isRunning;
public:
  Compressor()
    : isRunning(false) {}

  void start() override {
#ifdef DEBUG
    Serial.println("Counter Completed for Compressor");
    Serial.println("Compressor Turned On");
#endif
    compressor_timer_flag = false;
    compressor_onoff = 1;
    totalInterruptCounter = 0;
    timerAlarmDisable(Comp_on_off_timer);
    timerWrite(Comp_on_off_timer, 0);
    isRunning = true;
    Serial.println("Compressor started");
  }

  void stop() override {
#ifdef DEBUG
    Serial.println("Counter Completed for Turning OFF Compressor");
    Serial.println("Turned OFF Compressor");
#endif
    compressor_onoff = 0;
    totalInterruptCounter = 0;
    timerAlarmDisable(Comp_on_off_timer);
    timer1_flag = false;
    timerWrite(Comp_on_off_timer, 0);
    isRunning = false;
    Serial.println("Compressor stopped");
  }

  void checkStatus() override {}
  bool getStatus() {
    return isRunning;
  }
};

// ========== Solenoid Valve ==========
class SolenoidValve : public Component {
private:
  bool isOpen;
public:
  SolenoidValve()
    : isOpen(false) {}
  void start() override {
    isOpen = true;
    Serial.println("Valve Opened");
  }
  void stop() override {
    isOpen = false;
    Serial.println("Valve Closed");
  }
  void checkStatus() override {}
  // void updateStatus(float, int8_t) override {}
  bool getStatus() {
    return isOpen;
  }
};

// ========== EXV Class ==========
// class EXV : public Component {
// private:
//   int position;
// public:
//   EXV()
//     : position(0) {}
//   void start() override {
//     adjustPosition(25);
//   }
//   void stop() override {
//     adjustPosition(0);
//   }
//   void checkStatus() override {}
//   void adjustPosition(int newPosition) {
//     position = constrain(newPosition, 0, 100);
//     analogWrite(EXV_PIN, map(position, 0, 100, 0, 255));  // PWM control mockup
//     Serial.print("EXV adjusted to ");
//     Serial.println(position);
//   }
//   int getPosition() {
//     return position;
//   }
// };

// ========== PID for EXV ==========
// double setpoint = 12.0;  // Desired superheat
// double input, output;
// PID myPID(&input, &output, &setpoint, 2.0, 5.0, 1.0, DIRECT);

// ========== Instances ==========
// TemperatureSensor tempSensor1, tempSensor2, tempSensor3, tempSensor4, tempSensor5, tempSensor6, tempSensor7, tempSensor8;
// PressureSensor pressureSensor1, pressureSensor2;
// Compressor compressor1, compressor2;
// SolenoidValve valve1, valve1;
// EXV exv;

// ========== Setup ==========
// void setup() {
//   Serial.begin(115200);
//   pinMode(EXV_PIN, OUTPUT);

//   tempSensor.start();
//   pressureSensor.start();
//   compressor.start();
//   valve.start();
//   exv.start();

//   myPID.SetMode(AUTOMATIC);
// }

// ========== Mock Superheat Calculation ==========
float calculateSuperheat(float temp, float pressure) {
  // Simplified superheat calculation (replace with actual formula)
  return temp - (pressure * 0.2);
}

// ========== Loop ==========
// void loop() {
//   tempSensor.checkStatus();
// pressureSensor.checkStatus(1);

//   float temp = tempSensor.getTemperature();
//   float pressure = pressureSensor.getPressure();

//   input = calculateSuperheat(temp, pressure);
//   myPID.Compute();
//   exv.adjustPosition(output);

//   Serial.print("Temp: ");
//   Serial.print(temp);
//   Serial.print(" C, Pressure: ");
//   Serial.print(pressure);
//   Serial.print(" psi, Superheat: ");
//   Serial.print(input);
//   Serial.print(" C, EXV Pos: ");
//   Serial.println(exv.getPosition());

//   delay(1000);
// }
