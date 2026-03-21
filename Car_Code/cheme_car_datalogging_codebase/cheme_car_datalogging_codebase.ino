// McMaster Chem-E Car Team
// Chem-E Car Datalogging Code - 25/26

/*
The code to be used before the Chem-E Car competition to run the car while
logging relevant data for the Braking subteam. This will give them an idea
of consitency and performance from run to run. Makes use of an RP2040
microcontroller to drive the car and stop at a specified distance. More
information is available in the readme.
*/

// Remove macros
#undef radians
#undef degrees

// Included libraries
#include <Adafruit_BNO08x.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <BackgroundAudio.h>
#include <PWMAudio.h>
#include <SdFat.h>
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "encoder/encoder.cpp" // Modified to remove debounce delays, don't replace!
#include "encoder/encoder.hpp" // Modified to remove debounce delays, don't replace!

#define NUM_LEDS 1 // Status LED

// Define drive/brake motor pins
#define DRIVE_PIN 9
#define BRAKE_PIN 13

// Define the PWM pins for the stir bar motors
#define BRAK_STIR_PWM_1 A3
#define BRAK_STIR_PWM_2 24

// Define servo pins
#define BRAK_SERVO_PWM 11
#define PROP_SERVO_PWM 4
#define STEERING_SERVO_PWM 25

// Define Auxillary DC Motor Pins
#define AUXDC_PWM_1 6
#define AUXDC_PWM_2 5

// Define encoder pins
#define ENC_A A0
#define ENC_B A1

#define TURBIDITY_SENS A2 // Pin for the turbidity sensor data line(right encoder and braking sensor switched pins)
#define BNO08X_RESET -1   // No reset pin for IMU over I2C, only enabled for SPI
#define SD_CS_PIN 23      // Define chip select pin for SD card
#define A219_I2C 0x40     // I2C address for current/voltage sensor
#define AUDIO_OUT 12      // Define audio output pin

using namespace encoder;

// Struct for Euler Angles
struct euler_t
{
  float yaw;
  float pitch;
  float roll;
} ypr;

// Enumeration for door commands
enum door_cmd
{
  DOOR_STOP,
  DOOR_OPEN,
  DOOR_CLOSE
};

// Encoder constants
const float PPR = 8192.0;
const double WHEEL_CIRCUMFERENCE_M = 0.397883;

// Create servo objects
Servo brak_servo;
Servo prop_servo;
Servo steering_servo;

// Create encoder object using only A & B pins, not index, assign to pio0, sm 1 & 3 in normal direction w/ microstepping for smoothness
Encoder drive_encoder(PIO pio0, 3, {ENC_A, ENC_B}, PIN_UNUSED, NORMAL_DIR, PPR, true);

// Create BNO085 instance
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensor_value;

Adafruit_NeoPixel pixel(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800); // Status LED

// Define files
SdFat sd;
FsFile root;
FsFile next_file;
FsFile data_file;
FsFile audio_file;
String door_close_sound = "door_close_sound.mp3";
String time_circuit_sound = "time_circuit_sound.mp3";
String time_travel_sound = "time_travel_sound.mp3";
String door_open_sound = "door_open_sound.mp3";
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);
String file_name;

PWMAudio pwm(AUDIO_OUT);     // Set up audio output using PWM
BackgroundAudioMP3 mp3(pwm); // create the mp3 player object and decoder
uint8_t filebuff[512];       // allocates 512 bytes of memory to temporarily store audio data before processing

bool is_file_new = true; // Checks for new file

// Runtime flags
bool running = true;
bool audio_trig = false;
unsigned int audio_played = 0;

// The target yaw angle to keep car straight
const float GOAL_YAW = 0.0;

// Rejection threshold for noise in IMU measurements
const float REJECT_THRESHOLD = 3.0;

// define turbidity threshold for braking(THIS IS A PLACEHOLDER)
const float TURB_THRESHOLD = 3000.0; // THIS IS A PLACEHOLDER!!!

// Define voltage & current monitor variables
uint16_t raw_bus;
int16_t raw_current;
double bus_voltage;
double current_mA;

// Define IMU variables
double raw_yaw;        // raw yaw angle
double prev_yaw;       // previous unwrapped yaw angle
double yaw;            // unwrapped yaw angle
double init_yaw = 0.0; // initial yaw angle
double yaw_diff = 0.0; // yaw angle difference

// Wheel distance variables
double drive_dist_m = 0.0;

// Initialize run count for SD card file
int run_count;

double turbidity; // turbidity counts

// Keeping track of time
double curr_time = 0.0f;
double prev_time = 0.0f;
uint32_t start_time;

const int DATA_SIZE = 6; // Number of items to log
double data[DATA_SIZE];  // Data array

// PID loop variables
double error = 0.0;      // Proportional error
double last_error = 0.0; // Derivative error
double sum_error = 0.0;  // Integral error

// The following numbers need to be adjusted through testing
const float K_P = 25.0; // Proportional weighting
const float K_I = 0.0;  // Integral weighting
const float K_D = 0.0;  // Derivative weighting

// Offsets & constants for PID
const int SERVO_ANGLE = 1475;
const int MAX_OFFSET = 1024;
const int YAW_REF = 90;

/*
Description: Subroutine to close both drive and brake SSR.
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void reset_ssr()
{
  digitalWrite(DRIVE_PIN, LOW);
  digitalWrite(BRAKE_PIN, LOW);
}

/*
Description: Subroutine to drive car forward.
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void drive_ssr()
{
  reset_ssr();
  busy_wait_ms(1);
  digitalWrite(DRIVE_PIN, HIGH);
}

/*
Description: Subroutine to stop car and brake motors.
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void brake_ssr()
{
  reset_ssr();
  busy_wait_ms(1);
  digitalWrite(BRAKE_PIN, HIGH);
}

/*
Description: Subroutine to dump reactants into vessel.
Inputs:      void
Outputs:     void
Parameters:  (Servo)servo, (int)angle_us, (int)delay_ms
Returns:     void
*/
void servo_dump(Servo servo, int angle_us, int delay_ms)
{
  // Rotate to specified position
  servo.writeMicroseconds(angle_us);
  busy_wait_ms(500);
  servo.writeMicroseconds(angle_us);

  busy_wait_ms(delay_ms); // Wait specified delay

  // Return to default position
  servo.writeMicroseconds(450);
  busy_wait_ms(500);
  servo.writeMicroseconds(450);
}

/*
Description: Subroutine to set speed for reaction stir motors.
Inputs:      void
Outputs:     void
Parameters:  (int)stir_pin_1, (int)stir_pin_2, (int)speed
Returns:     void
*/
void start_stir(int stir_pin_1, int stir_pin_2, int speed)
{
  digitalWrite(stir_pin_1, LOW); // For fast decay
  analogWrite(stir_pin_2, 255);  // Set motor to max for a kickstart
  busy_wait_ms(1000);
  digitalWrite(stir_pin_1, LOW);  // For fast decay
  analogWrite(stir_pin_2, speed); // Set motor to speed obtained through testing
}

/*
Description: Subroutine to stop reaction stir motors.
Inputs:      void
Outputs:     void
Parameters:  (int)stir_pin_1, (int)stir_pin_2
Returns:     void
*/
void stop_stir(int stir_pin_1, int stir_pin_2)
{
  digitalWrite(stir_pin_1, LOW); // For fast decay
  digitalWrite(stir_pin_2, LOW); // Stop motor
}

/*
Description: Subroutine to print relevant data out to serial or a microSD card.
Inputs:      void
Outputs:     void
Parameters:  (bool)serial_true, (double)millis_time, (double)outputs[DATA_SIZE]
Returns:     void
*/
void printer(bool serial_true, double millis_time, double outputs[DATA_SIZE])
{
  if (serial_true) // Print data to serial or SD card file accordingly in .csv format
  {
    Serial.print(millis_time, 6);

    for (int i = 0; i < DATA_SIZE; i++)
    {
      Serial.print(",");
      Serial.print(outputs[i], 6);
    }

    Serial.println("");
  }
  else
  {
    data_file.print(millis_time, 6);

    for (int i = 0; i < DATA_SIZE; i++)
    {
      data_file.print(",");
      data_file.print(outputs[i], 6);
    }

    data_file.println("");
  }
}

/*
Description: Subroutine to command the door motor
Inputs:      void
Outputs:     void
Parameters:  (door_cmd)cmd, (int)speed
Returns:     void
*/
void door_motor(door_cmd cmd, int speed)
{
  switch (cmd)
  {
  case DOOR_STOP:
    analogWrite(AUXDC_PWM_1, 0);
    analogWrite(AUXDC_PWM_2, 0);
    break;
  case DOOR_OPEN:
    analogWrite(AUXDC_PWM_1, 0);
    analogWrite(AUXDC_PWM_2, speed);
    break;
  case DOOR_CLOSE:
    analogWrite(AUXDC_PWM_1, speed);
    analogWrite(AUXDC_PWM_2, 0);
    break;
  default:
    break;
  }
}

/*
Description: Subroutine to set the desired reports on the IMU.
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void set_reports(void)
{
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR))
  {
    Serial.println("Could not enable rotation vector");
  }
}

/*
Description: Subroutine convert quaternions from the IMU into euler angles to obtain yaw.
Inputs:      void
Outputs:     void
Parameters:  (float)qr, (float)qi, (float)qj, (float)qk, (euler_t)*ypr, (bool)degrees
Returns:     void
*/
void quaternion_to_euler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees = false)
{

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees)
  {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

/*
Description: Pointer subroutine for euler angles.
Inputs:      void
Outputs:     void
Parameters:  (sh2_RotationVectorWAcc_t)*rotational_vector, (float)qi, (euler_t)*ypr, (bool)degrees
Returns:     void
*/
void quaternion_to_euler_RV(sh2_RotationVectorWAcc_t *rotational_vector, euler_t *ypr, bool degrees = false)
{
  quaternion_to_euler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

/*
Description: PID loop subroutine to adjust steering angle.
Inputs:      (double)error, (double)sum_error, (double)last_error, (double)curr_time, (double)prev_time
Outputs:     (int)adj_pid_output
Parameters:  void
Returns:     void
*/
void pid_loop(void)
{
  // Generate output of controller based on constants & errors
  double pid_output = (error * K_P + sum_error * K_I + (error - last_error) / (curr_time - prev_time) * K_D) / YAW_REF * MAX_OFFSET;
  int adj_pid_output = max(min(round(pid_output), MAX_OFFSET), -MAX_OFFSET); // Clamp output to bounds

  // Get IMU reading to update error values
  bno08x.getSensorEvent(&sensor_value);
  quaternion_to_euler_RV(&sensor_value.un.rotationVector, &ypr, true);

  // Process yaw angle
  raw_yaw = ypr.yaw;
  unwrap_yaw();
  yaw_diff = yaw - init_yaw;

  // Update errors
  last_error = error;
  error = GOAL_YAW - yaw_diff;
  sum_error = max(min(sum_error + cbrt(error), MAX_OFFSET), -MAX_OFFSET);

  // Write to servos
  steering_servo.writeMicroseconds(SERVO_ANGLE + adj_pid_output);
}

/*
Description: Subroutine to fetch turbidity samples and output the average value.
Inputs:      void
Outputs:     void
Parameters:  (int)samples
Returns:     (float)turbidity
*/
float fetch_turb(int samples)
{
  // calculate turbidity (avg of # measurements)
  float turbidity = 0.0f;

  for (int i = 0; i < samples; i++)
  {
    turbidity += (double)analogRead(TURBIDITY_SENS);
  }
  turbidity = turbidity / (double)samples;

  return turbidity;
}

/*
Description: Subroutine to unwrap yaw angle to prevent discontinuities in the function over time.
Inputs:      (double)raw_yaw, (double)prev_yaw
Outputs:     (double)yaw
Parameters:  void
Returns:     void
*/
void unwrap_yaw(void)
{
  double delta = raw_yaw - prev_yaw;

  delta = fmod(delta + 180.0, 360.0);

  if (delta < 0.0)
  {
    delta += 360.0;
  }

  delta -= 180.0;

  // Reject any sudden noisy spikes
  if (fabs(delta) > REJECT_THRESHOLD)
  {
    yaw = prev_yaw;
    prev_yaw = yaw;
  }
  else
  {
    yaw = prev_yaw + delta;
    prev_yaw = yaw;
  }
}

/*
Description: Turn off speaker when not in use to prevent overheating and ringing noise
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void start_speaker(void)
{
  pwm.begin(AUDIO_OUT);
  mp3.begin();
}

/*
Description: Turn on speaker before playing audio
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void stop_speaker(void)
{
  pwm.end();
  mp3.end();
}

/*
Description: Send audio data chunks to the decoder and close file when done
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void send_audio(void)
{
  // If an audio file is open and the decoder buffer has enough space for more audio data
  if (audio_file && mp3.availableForWrite() > 512)
  {
    // read 512 bytes from the audio file and send to the decoder
    int len = audio_file.read(filebuff, 512);
    mp3.write(filebuff, len);

    // if the audio data was shorter than 512 bytes, we've reached the end of the file
    if (len != 512)
    {
      audio_trig = false;
      audio_played++;

      audio_file.close();
      stop_speaker();
    }
  }
}

/*
Description: Helper function to write value to register over I2C
Inputs: void
Outputs: void
Parameters: (uint8_t)address, (uint8_t)reg, (uint16_t)value
Returns: void
*/
void write_register(uint8_t address, uint8_t reg, uint16_t value)
{
  Wire.beginTransmission(address);

  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF); // MSB
  Wire.write(value & 0xFF);        // LSB

  Wire.endTransmission();
}

/*
Description: Helper function to read value from register over I2C
Inputs: void
Outputs: void
Parameters: (uint8_t)address, (uint8_t)reg
Returns: (uint16_t)value
*/
uint16_t read_register(uint8_t address, uint8_t reg)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)2); // Request 2 bytes

  // If data available then return it
  if (Wire.available() >= 2)
  {
    uint16_t value = Wire.read() << 8;
    value |= Wire.read();
    return value;
  }

  return 0; // Otherwise return 0
}

/*
Description: Arduino setup subroutine.
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void setup(void)
{
  stop_speaker(); // Don't overheat speaker

  // Intitalize Voltage/Current Sensor
  Wire.begin();

  /*
  Write to config register (0x00)
  Sets it to 16V bus range
  Gain /4
  12 bit ADC
  Continuous Mode
  */
  // 8-sample averaging
  // Needs about 5ms to process next batch of samples
  write_register(A219_I2C, 0x00, 0x15DF);

  /*
  Write to calibration register (0x05)
  Assumes 0.1 Ohm resistor and 2.0A max current (0.1 mA per bit)
  Uses formula 0.04096/(Resistor*Current_Per_Bit)
  Writes 4096 in this case
  */
  write_register(A219_I2C, 0x05, 0x1000);

  // --- READ BUS VOLTAGE (0x02) ---
  raw_bus = read_register(A219_I2C, 0x02);
  // Shift right 3 bits to remove status flags, then multiply by 4mV
  bus_voltage = (raw_bus >> 3) * 0.004;

  // --- READ CURRENT (0x04) ---
  raw_current = (int16_t)read_register(A219_I2C, 0x04);
  // Multiply by calculated LSB (0.1mA)
  current_mA = raw_current * 0.1;

  // Indicate status to be initialized
  pixel.begin();
  pixel.setBrightness(255);
  pixel.show();
  pixel.setPixelColor(0, 255, 0, 0);
  pixel.show();

  mp3.begin(); // Initialize mp3 module
  Serial.begin(115200);

  // Initialize SD card with blocking to ensure data is logged before run is started
  while (!sd.begin(config))
  {
    busy_wait_ms(1000); // Wait for a second before retrying
  }

  root = sd.open("/", FILE_READ); // Open SD root directory
  run_count = -4;

  while (true)
  {
    next_file = root.openNextFile();

    if (next_file)
    {
      run_count++; // Increment with each existing file
    }
    else
    {
      next_file.close();
      break;
    }
  }

  root.close();

  // Set file numbering
  file_name = "Run_" + String(run_count) + ".csv";

  // Setting to drive,brake motors output mode
  pinMode(DRIVE_PIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);

  brake_ssr();    // Stop driving motors from any residual bootloader code

  // Initialize the auxiliary DC motor pins as outputs
  pinMode(BRAK_STIR_PWM_1, OUTPUT);
  pinMode(BRAK_STIR_PWM_2, OUTPUT);
  pinMode(AUXDC_PWM_1, OUTPUT);
  pinMode(AUXDC_PWM_2, OUTPUT);

  // Setting the stir speed
  start_stir(BRAK_STIR_PWM_1, BRAK_STIR_PWM_2, 154);

  analogReadResolution(12);
  pinMode(TURBIDITY_SENS, INPUT);
  bno08x.begin_I2C();
  set_reports();

  // Poll IMU a few times & wait for initialization value to stabilize
  for (int i = 0; i < 5; i++)
  {
    bno08x.getSensorEvent(&sensor_value);
    quaternion_to_euler_RV(&sensor_value.un.rotationVector, &ypr, true);

    init_yaw = ypr.yaw;
    raw_yaw = init_yaw;
    prev_yaw = init_yaw;
    yaw = init_yaw;
    yaw_diff = yaw - init_yaw;

    busy_wait_ms(200);
  }

  door_motor(DOOR_CLOSE, 154); // Start closing door

  // Play sound effect for doors
  if (audio_played == 0)
  {
    audio_file = sd.open(door_close_sound, FILE_READ); // Open corresponding SD card mp3 file
    start_speaker();
  }

  while (audio_file) // If the audio file opened successfully
  {
    send_audio(); // If the audio file has been opened and is still open, send an audio data chunk
  }

  door_motor(DOOR_STOP, 0); // Stop closing door

  // Initialize servos to default position
  prop_servo.writeMicroseconds(450);
  prop_servo.attach(PROP_SERVO_PWM, 400, 2600);
  prop_servo.writeMicroseconds(450);
  busy_wait_ms(2000);
  brak_servo.writeMicroseconds(450);
  brak_servo.attach(BRAK_SERVO_PWM, 400, 2600);
  brak_servo.writeMicroseconds(450);
  busy_wait_ms(2000);
  steering_servo.writeMicroseconds(1475);
  steering_servo.attach(STEERING_SERVO_PWM, 400, 2600);
  steering_servo.writeMicroseconds(1475);
  busy_wait_ms(2000);

  // Dump reactants before starting drive
  servo_dump(prop_servo, 2500, 3000);

  // Wait for busVolatge to surpass 7V
  // while (bus_voltage < 7)
  // {
  //   raw_bus = read_register(A219_I2C, 0x02);
  //   bus_voltage = (raw_bus >> 3) * 0.004;
  // }

  // Play sound effect for time circuit initialization
  if (audio_played == 1)
  {
    audio_file = sd.open(time_circuit_sound, FILE_READ); // Open corresponding SD card mp3 file
    start_speaker();
  }

  while (audio_file) // If the audio file opened successfully
  {
    send_audio(); // If the audio file has been opened and is still open, send an audio data chunk
  }

  servo_dump(brak_servo, 2500, 3000);

  start_time = time_us_32(); // First measurement saved seperately

  // busy_wait_ms(17000);

  // Poll IMU one last time
  bno08x.getSensorEvent(&sensor_value);
  quaternion_to_euler_RV(&sensor_value.un.rotationVector, &ypr, true);

  init_yaw = ypr.yaw;
  raw_yaw = init_yaw;
  prev_yaw = init_yaw;
  yaw = init_yaw;
  yaw_diff = yaw - init_yaw;

  // Initialize encoders & zero before starting
  drive_encoder.init();
  drive_encoder.zero();

  curr_time = (time_us_32() - start_time) / 1000000.0f; // Taken to update prev_time

  pixel.setPixelColor(0, 0, 0, 255); // Indicate setup complete status
  pixel.show();

  drive_ssr(); // Start drive
}

/*
Description: Arduino loop subroutine.
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void loop(void)
{
  prev_time = curr_time;

  turbidity = fetch_turb(20); // fetch turbidity measurement

  curr_time = (time_us_32() - start_time) / 1000000.0f; // Taken to check time against first measurement

  if (running)
  {
    pid_loop(); // Run PID controller
  }

  // Convert encoder counts to distance
  drive_dist_m = (double)drive_encoder.count() / PPR * WHEEL_CIRCUMFERENCE_M;

  raw_bus = read_register(A219_I2C, 0x02);
  bus_voltage = (raw_bus >> 3) * 0.004;

  raw_current = (int16_t)read_register(A219_I2C, 0x04);
  current_mA = raw_current * 0.1;

  // If outside of 3-14V range of > 1A current draw then stop
  // if (bus_voltage > 14 || bus_voltage < 3 || current_mA > 1000)
  // {
  //   brake_ssr();

  //   pixel.setPixelColor(0, 255, 0, 0); // Turn LED to red
  //   pixel.show();
  // }

  // Update data array
  data[0] = turbidity;
  data[1] = yaw;
  data[2] = yaw_diff;
  data[3] = drive_dist_m;
  data[4] = current_mA;
  data[5] = bus_voltage;

  // Open csv file
  data_file = sd.open(file_name, FILE_WRITE);

  // Write to csv file
  if (data_file)
  {
    // Write file header
    if (is_file_new)
    {
      data_file.println("Time (s),Turbidity (counts),Raw Yaw Angle (deg),Delta Yaw Angle (deg),Wheel Distance (m),Current (mA),Voltage (V)");
      is_file_new = false;
    }

    printer(false, curr_time, data); // Write variable data to the file in CSV format

    data_file.close();
  }

  printer(true, curr_time, data); // Write variable data to serial in CSV format

  // if (turbidity >= TURB_THRESHOLD && running)
  // {
  //   running = false; // Set flag

  //   brake_ssr();                                 // Stop driving
  //   stop_stir(BRAK_STIR_PWM_1, BRAK_STIR_PWM_2); // Stop stirring motor
  // }

  // if (!running)
  // {
  //   if (audio_played == 2 && !audio_trig) // Play sound effect for time travel
  //   {
  //     audio_file = sd.open(time_travel_sound, FILE_READ);
  //     start_speaker();
  //     audio_trig = true;
  //   }
  //   else if (audio_played == 3 && !audio_trig) // Play sound effect for doors
  //   {
  //     // Indicate status to be finished
  //     pixel.setPixelColor(0, 0, 255, 0);
  //     pixel.show();

  //     door_motor(DOOR_OPEN, 154); // Start opening door

  //     audio_file = sd.open(door_open_sound, FILE_READ);
  //     start_speaker();
  //     audio_trig = true;
  //   }
  //   else if (audio_played == 4 && !audio_trig) // Done everything, run once
  //   {
  //     door_motor(DOOR_STOP, 0); // Stop opening door
  //     audio_trig = true;
  //   }

  //   // keep playing audio until file is closed
  //   if (audio_file)
  //   {
  //     send_audio();
  //   }
  // }
}
