// Define the I2C address for the PCF8574
#define In_ADDRESS 0x27
#define Out_ADDRESS 0x26

// Function to {write 0-7 outputs to the PCF8574 from the provided array
void writeOutputs(bool out[8]) {
  // digitalWrite(25,HIGH);
  // digitalWrite(26,HIGH);
  // digitalWrite(27,HIGH);
  // digitalWrite(13,HIGH);
  // out[0] = compressor_onoff;
  // out[1] = sv_l_u;
  // out[2] = condenser_fan_1;
  // out[3] = condenser_fan_2;
  uint8_t data = 0;
  if (write_compressors == false) {      // Write on EXV OUTPUT
    for (int i = 0; i < 8; i = i + 1) {  // i=i+1
      if (out[i] == 1) {
        data |= (1 << i);
      }
    }
  }
  Wire.beginTransmission(Out_ADDRESS);
  Wire.write(data);
  Wire.endTransmission();
}

// Function to read all inputs on the PCF8574
void readInputs(bool in[8]) {
  uint8_t data = 0;
  Wire.beginTransmission(In_ADDRESS);
  Wire.endTransmission();
  // Request 1 byte from the I2C device
  Wire.requestFrom(In_ADDRESS, (int8_t)1);

  if (Wire.available()) {
    data = Wire.read();
  }
  for (int i = 0; i < 8; i++) {
    in[i] = (data >> i) & 1;
  }
  // Update inputs array with manually set values
  // start = in[0];
  inner_fan_sw = in[0];
  stop = in[1];
  es = in[2];
  spare = in[3];
  sl_ps = in[4];
  dl_ps = in[5];
  o_ps = in[6];
  pf_relay = in[7];

#ifdef DEBUG
  // Serial.print("start = ");
  // Serial.println(start);
  // Serial.print("stop = ");
  // Serial.println(stop);
  // Serial.print("es = ");
  // Serial.println(es);
  // Serial.print("spare = ");
  // Serial.println(spare);
  // Serial.print("sl_ps = ");
  // Serial.println(sl_ps);
  // Serial.print("dl_ps = ");
  // Serial.println(dl_ps);
  // Serial.print("o_ps = ");
  // Serial.println(o_ps);
  // Serial.print("pf_relay = ");
  // Serial.println(pf_relay);
#endif
}

// Function to initialize the PCF8574 and check if it is connected
bool initializePCF8574(int PCF8574_ADDRESS_1, int PCF8574_ADDRESS_2) {
  bool flag = false;
  int id = 0;
  Wire.beginTransmission(PCF8574_ADDRESS_1);
  uint8_t error_1 = Wire.endTransmission();
  if (error_1 == 0) {
    flag = true;  // PCF8574 is connected and responded
  } else {
    flag = false;  // PCF8574 did not respond
    id = 1;
  }
  if (flag == false) {
    Wire.beginTransmission(PCF8574_ADDRESS_2);
    uint8_t error_2 = Wire.endTransmission();
    if (error_2 == 0) {
      flag = true;  // PCF8574 is connected and responded
    } else {
      flag = false;  // PCF8574 did not respond
      id = 2;
    }
  }
  return flag;
}

void write_on_EXV(int no_of_steps) {
  // number_of_steps = no_of_steps;
  if (currentStep < no_of_steps && pulse_count == false) {

    if (currentStep < 126) {
      pulse_count = true;
      move_direction = true;
      currentStep++;
    }  //
    else {
      pulse_count = false;
    }
  }  //
  else if (currentStep > no_of_steps && pulse_count == false) {

    if (currentStep > 0) {
      pulse_count = true;
      move_direction = false;
      currentStep--;
    }  //
    else {
      pulse_count = false;
    }
  }

  // Serial.println("write_on_EXV");

  if (millis() - write_exv_delay >= pulse_delay) {
    write_exv_delay = millis();
    // writeOutputs(Output_Data_2);
    // Output_Data_2[4] = 1;
    // Output_Data_2[5] = 1;
    // Output_Data_2[6] = 1;
    // Output_Data_2[7] = 1;


    // digitalWrite(25, HIGH);
    // digitalWrite(26, HIGH);
    // digitalWrite(27, HIGH);
    // digitalWrite(13, HIGH);

    if (pulse_count == true && move_direction == true) {

      if (pulse > 4) {

        if (currentStep - prev_steps == 10) {
          preferences.begin("EXV_Steps", false);
          preferences.putInt("currentStep", currentStep);
          preferences.end();

#ifdef DEBUG
          counter++;
          Serial.println("Saved currentStep - prev_steps");
#endif

          prev_steps = currentStep;
        }

        pulse = 0;
      }
      pulse++;
      switch (pulse) {

        case 1:
          Output_Data_2[4] = 0;

          digitalWrite(EXV_1, LOW);
          digitalWrite(EXV_2, HIGH);
          digitalWrite(EXV_3, HIGH);
          digitalWrite(EXV_4, HIGH);

          break;

        case 2:
          Output_Data_2[5] = 0;

          digitalWrite(EXV_1, HIGH);
          digitalWrite(EXV_2, LOW);
          digitalWrite(EXV_3, HIGH);
          digitalWrite(EXV_4, HIGH);
          break;

        case 3:
          Output_Data_2[6] = 0;

          digitalWrite(EXV_1, HIGH);
          digitalWrite(EXV_2, HIGH);
          digitalWrite(EXV_3, LOW);
          digitalWrite(EXV_4, HIGH);
          break;

        case 4:
          Output_Data_2[7] = 0;

          digitalWrite(EXV_1, HIGH);
          digitalWrite(EXV_2, HIGH);
          digitalWrite(EXV_3, HIGH);
          digitalWrite(EXV_4, LOW);
          pulse_count = false;
          break;

        default:
          break;
      }
    } else if (pulse_count == true && move_direction == false) {
      if (pulse > 4) {
        if (prev_steps - currentStep == 10) {

          preferences.begin("EXV_Steps", false);
          preferences.putInt("currentStep", currentStep);
          preferences.end();
#ifdef DEBUG
          counter++;
          Serial.println("Saved prev_steps - currentStep");
#endif
          prev_steps = currentStep;
        }
        // preferences.begin("EXV_Steps", false);
        // preferences.putInt("currentStep", currentStep);
        // preferences.end();
        pulse = 0;
      }
      pulse++;
      switch (pulse) {

        case 1:
          Output_Data_2[7] = 0;

          digitalWrite(EXV_1, HIGH);
          digitalWrite(EXV_2, HIGH);
          digitalWrite(EXV_3, HIGH);
          digitalWrite(EXV_4, LOW);
          break;

        case 2:
          Output_Data_2[6] = 0;

          digitalWrite(EXV_1, HIGH);
          digitalWrite(EXV_2, HIGH);
          digitalWrite(EXV_3, LOW);
          digitalWrite(EXV_4, HIGH);
          break;

        case 3:
          Output_Data_2[5] = 0;

          digitalWrite(EXV_1, HIGH);
          digitalWrite(EXV_2, LOW);
          digitalWrite(EXV_3, HIGH);
          digitalWrite(EXV_4, HIGH);
          break;

        case 4:
          Output_Data_2[4] = 0;

          digitalWrite(EXV_1, LOW);
          digitalWrite(EXV_2, HIGH);
          digitalWrite(EXV_3, HIGH);
          digitalWrite(EXV_4, HIGH);
          pulse_count = false;
          break;

        default:
          break;
      }
    }
  }
}