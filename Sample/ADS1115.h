#include <Wire.h>
#include <Adafruit_ADS1X15.h>

/*
  Voltage= ADC value × full-scale voltage /  resolution
  where :
  FULL-SCALE VOLTAGE => 4.096V
  RESOLUTION => 2^16 / 2 = 32768 i.e MAX ADC VALUE => 32768
*/

#define SENSOR_MAX_ANALOG 23100  // 20mA / 3.0V
#define SENSOR_MIN_ANALOG 4800   // 4mA / 0.6V
#define SENSOR_MIN 0
#define SENSOR_MAX 580  // in psi since sensor is of 40bar so 40bar = 580psi
#define ADC_Ports 4     // 4 Channels for each ADS1115

// Adafruit_ADS1115 ads1(0x48); /* Use this for the 16-bit version */
// Adafruit_ADS1115 ads2(0x49); /* Use this for the 16-bit version */

Adafruit_ADS1115 ads1; /* Use this for the 16-bit version */
Adafruit_ADS1115 ads2; /* Use this for the 16-bit version */

float discharge = 0, suction = 0;

bool initializeADS1115(int ADS_ADDRESS) {
  Wire.beginTransmission(ADS_ADDRESS);
  uint8_t error = Wire.endTransmission();
  if (error == 0) {
    return true;  // A_D_S_1115  is connected
  } else {
    return false;  // A_D_S_1115  did not respond
  }
}

void ADSinit() {
  if (initializeADS1115(0x48) && initializeADS1115(0x49)) {
    Serial.println("ads1115 Initialized Successfully!");
    ads1.setGain(GAIN_ONE);  // 1x gain (±4.096V)
    ads1.begin();
    ads2.setGain(GAIN_ONE);  // 1x gain (±4.096V)
    ads2.begin();
  }
}

// 10040 - 1.25v
// 26060 - 3.25v
// volt = (Analog Value / 32768) * 4.096

void scanADC() {
  float value;
  for (int channel = 0; channel < ADC_Ports; channel++) {
    long raw = 0;
    for (int j = 0; j < 10; j++) {
      raw += ads1.readADC_SingleEnded(channel);
      delayMicroseconds(500);  // Small delay to reduce noise
    }
    raw /= 10;  // Average of 10 samples
    raw = constrain(raw, SENSOR_MIN_ANALOG, SENSOR_MAX_ANALOG);
    value = map(raw, SENSOR_MIN_ANALOG, SENSOR_MAX_ANALOG, SENSOR_MIN, SENSOR_MAX);

    switch (channel) {
      case 0:
        suction = value;  // AI1
        break;
      case 1:
        discharge = value;  // AI0
        break;
      default:
        break;
    }
  }
  // Serial.println();
}



// Serial.print("CH");
// Serial.print(channel);
// Serial.print(" : ");
// Serial.print(raw);
// float voltage = raw * 4.096 / 32768.0;
// Serial.print("  ");
// Serial.print(voltage);
// Serial.print("  ");
// Serial.print("  ");
// Serial.print(value);
// Serial.print("\t");
