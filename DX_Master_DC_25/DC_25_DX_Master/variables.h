// Sensors Outputs
float sl_temp, d_temp, sp_temp, sa_temp, ra_temp;
float sl_temp_by_pres;

// Pressures Inputs
float sl_pres, dl_pres;

// Pressure Switches and Phase Failure Relay
bool start, stop, es, spare, sl_ps, dl_ps, o_ps, pf_relay, inner_fan_sw;
bool inputs[8] = { start, stop, es, spare, sl_ps, dl_ps, o_ps, pf_relay };

// Outputs;
uint8_t compressor_onoff, sv_l_u, condenser_fan_1, condenser_fan_2;
bool Output_Data_2[8] = { compressor_onoff, sv_l_u, condenser_fan_1, condenser_fan_2, 0, 0, 0, 0 };

int16_t pulse_delay = 50;

int16_t power_on_pres = 125;
int16_t cut_off_pres = 100;

int8_t Low_Temperature_Alarm = 20;
uint16_t High_Temperature_Alarm = 150;
uint16_t Spray_Temperature_Alarm = 50;
int8_t Low_Pressure_Alarm = 20;
int8_t High_Pressure_Alarm = 600;

unsigned long last_read_time, write_exv_delay = 0;

uint8_t currentStep = 0;
uint8_t prev_steps = 0;
uint8_t no_of_times_global = 0;
bool write_compressors = false;

bool exv_flagg, exv_position = true;
uint8_t steps_exv = 0;
#define EXV_1 25
#define EXV_2 26
#define EXV_3 27
#define EXV_4 13

unsigned long write_on_comp = 0;
uint8_t compressor_on_delay = 5;
uint16_t compressor_off_delay = 10;
uint8_t solenoid_valve_on_delay = 5;
bool solenoid_on_timer_flag, timer1_flag, compressor_timer_flag = false;

volatile int interruptCounter = 1;    //for counting interrupt0
volatile int interruptCounter_2 = 1;  //for counting interrupt
int8_t totalInterruptCounter;         //total interrupt counting
int8_t totalInterruptCounter_2;       //total interrupt counting
unsigned long saved_hours;


hw_timer_t *Comp_on_off_timer, *solenoid_valve_timer = NULL;  //H/W timer defining (Pointer to the Structure)

// float Return_Air_Temperature = 18.0;
float r_a_temp_Setpoint = 25.0;

enum init_exv : uint8_t {
  reset,
  initializing,
  ready
};

init_exv exv_state = reset;

uint8_t pulse = 0;
bool pulse_count = true;  // True for Close and False for Open direction
bool move_direction = true;
uint8_t number_of_steps;

//Specify the links and initial tuning parameters
double Setpoint = 25, Input, Output, new_setpoint;
double Kp = 2, Ki = 0, Kd = 0;
float Suction_Super_Heat;
const int16_t MAX_STEPS = 125;

unsigned long wait_time;
unsigned long update_values;


uint16_t* float_As_Registers;  // Interpret float as uint16_t

int8_t gas_selected_s = 0;
int8_t gas_selected_d = 0;

bool condenser_fan_enable = true;
bool condenser_fan_disable = false;


// int counter = 0;

bool low_temp_alarm = true;
bool high_temp_alarm = true;
bool low_pres_alarm = true;
bool high_pres_alarm = true;
bool stop_alarm = true;
bool inner_fan_alarm = true;
// int8_t crc_error_count = 0;
int8_t crc_error_count = 0;
bool vfd_start = false;

/* Modbus Variables*/


#define MODBUS_BAUD_RATE 9600
#define RS485_RX_PIN 16
#define RS485_TX_PIN 17
#define RS485_DE_RE_PIN 4  // DE/RE pin for controlling TX/RX mode
#define MODBUS_SLAVE_ID 1
#define MODBUS_FUNCTION_READ_HOLDING_REGISTERS 0x03
#define MODBUS_FUNCTION_WRITE_SINGLE_REGISTER 0x06
