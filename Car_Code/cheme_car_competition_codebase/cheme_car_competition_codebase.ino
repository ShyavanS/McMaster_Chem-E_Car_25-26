// McMaster Chem-E Car Team
// Chem-E Car Competition Code - 25/26

/*
The code to be used at the Chem-E Car competition to run the car. Makes use of
an RP2040 microcontroller to drive the car and stop at a specified distance.
More information is available in the readme.
*/

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

#define NUM_LEDS 1    // Status LED
#define STRIP_LEDS 22 // LED Strip

// Define drive and brake SSR pins
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

#define STRIP_PIN 10 // Light pin

#define TURBIDITY_SENS A2 // Pin for the turbidity sensor data line(right encoder and braking sensor switched pins)
#define BNO08X_RESET -1   // No reset pin for IMU over I2C, only enabled for SPI
#define SD_CS_PIN 23      // Define chip select pin for SD card
#define A219_I2C 0x40     // I2C address for current/voltage sensor
#define AUDIO_OUT 12      // Define audio output pin

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

// Enumeration for multicore commands
enum mutlicore_cmd
{
  PLAY_DOOR_OPEN = 1,
  PLAY_DOOR_CLOSE,
  PLAY_TIME_TRAVEL,
  PLAY_TIME_CIRCUIT,
  STATUS_ERROR,
  STATUS_STARTED,
  STATUS_STOPPED,
  SD_INITIALIZED
};

// Create servo objects
Servo brak_servo;
Servo prop_servo;
Servo steering_servo;

// Create BNO085 instance
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensor_value;

Adafruit_NeoPixel pixel(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800); // Status LED
Adafruit_NeoPixel strip(STRIP_LEDS, STRIP_PIN, NEO_GRB + NEO_KHZ800);  // Strip LEDs

// Define files
SdFat sd;
FsFile audio_file;
String door_close_sound = "door_close_sound.mp3";
String time_circuit_sound = "time_circuit_sound.mp3";
String time_travel_sound = "time_travel_sound.mp3";
String door_open_sound = "door_open_sound.mp3";
String theme_music = "theme_music.mp3";
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);
String file_name;

// Set up audio output using PWM
PWMAudio pwm(AUDIO_OUT);
BackgroundAudioMP3 mp3(pwm); // create the mp3 player object and decoder
uint8_t filebuff[512];       // allocates 512 bytes of memory to temporarily store audio data before processing

// LED variables
bool breath_status = false;
bool fluxing = false;
bool time_jump = false;
int breath_brightness = 0;
int t = 0;
uint32_t tic;
uint32_t toc;

bool sd_init = false; // SD intialization flag

// Runtime flags
volatile bool running = true;
volatile bool audio_trig = false;
volatile unsigned int audio_played = 0;

// The target yaw angle to keep car straight
const float GOAL_YAW = 0.0;

// Rejection threshold for noise in IMU measurements
const float REJECT_THRESHOLD = 3.0;

// define turbidity threshold for braking
const float TURB_THRESHOLD = 3500.0;

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

double turbidity; // turbidity counts

// Keeping track of time
double curr_time = 0.0f;
double prev_time = 0.0f;
uint32_t start_time;

// PID loop variables
double error = 0.0;      // Proportional error
double last_error = 0.0; // Derivative error
double sum_error = 0.0;  // Integral error

// Current values obtained through Ziegler–Nichols method with K_u = 30 & T_u = 266.7 ms.
// Divide derivative term by 100 to make sense on this timescale
const float K_P = 18.0;  // Proportional weighting
const float K_I = 0.135; // Integral weighting
const float K_D = 0.612; // Derivative weighting

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
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR))
  {
    exit(0);
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
  sum_error = max(min(sum_error + error, MAX_OFFSET), -MAX_OFFSET);

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
Description: Turn on speaker before playing audio
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void stop_speaker(void)
{
  while (!audio_file && !mp3.done())
    ;

  memset(filebuff, 0, 512);
  mp3.write(filebuff, 512);
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
Description: Activates Orange/Blue car lights
Inputs: void
Outputs: void
Parameters: void
Returns: void
*/
void car_lights(void)
{
  if (!breath_status)
  {
    breath_brightness += 10;

    if (breath_brightness >= 255)
    {
      breath_brightness = 255;
      breath_status = true;
    }

    for (int i = 1; i < STRIP_LEDS; i++)
    {
      strip.setPixelColor(i, 0, (int)255 * breath_brightness / 255, (int)255 * breath_brightness / 255); // set rest of pixels blue for now...
    }
  }
  else
  {
    int spark = random(0, 50);

    if (spark < 5)
    {
      for (int i = 1; i < STRIP_LEDS; i++)
      {
        strip.setPixelColor(i, 255, 40, 0); // set rest of pixels orange for now...
      }
    }
    else
    {
      for (int i = 1; i < STRIP_LEDS; i++)
      {
        strip.setPixelColor(i, 0, 255, 255); // set rest of pixels blue for now...
      }
    }
  }

  strip.setPixelColor(0, 255, 255, 255);
  strip.show();
}

/*
Description: Turns on flux capacitor light
Inputs: void
Outputs: void
Parameters: void
Returns: void
*/
void flux_cap(void)
{
  t++;

  // Base sinusoid + flicker
  float wave = (sin(t * 0.1) + 1) * 0.5;
  int base = 120 + wave * 100;
  int flicker = random(-40, 40);
  int b = constrain(base + flicker, 0, 255);

  // Sudden burst for extra random behaviour
  int burst = random(0, 50);

  if (burst < 10)
  {
    b = 255;
  }
  else if (burst > 40)
  {
    b = 0;
  }

  strip.setPixelColor(0, b, b, b);
  strip.show();
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
Description: Arduino core 0 setup subroutine.
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void setup(void)
{
  // Intitalize Voltage/Current Sensor
  Wire.begin();

  // Write to config register (0x00), Sets it to 16V bus range, Gain /4, 12 bit ADC, Continuous Mode, 8-sample averaging
  write_register(A219_I2C, 0x00, 0x15DF);

  // Write to calibration register (0x05), 0.1 Ohm resistor and 2.0A max current (0.1 mA per bit)
  // Uses formula 0.04096/(Resistor*Current_Per_Bit), 12 bit scaling
  write_register(A219_I2C, 0x05, 0x1000);

  // --- READ BUS VOLTAGE (0x02) ---
  raw_bus = read_register(A219_I2C, 0x02);
  // Shift right 3 bits to remove status flags, then multiply by 4mV
  bus_voltage = (raw_bus >> 3) * 0.004;

  // --- READ CURRENT (0x04) ---
  raw_current = (int16_t)read_register(A219_I2C, 0x04);
  // Multiply by calculated LSB (0.1mA)
  current_mA = raw_current * 0.1;

  // Block if SD not initialized for 3 seconds, then move on
  uint32_t cmd = 0;
  start_time = time_us_32();

  while (!rp2040.fifo.pop_nb(&cmd) && time_us_32() - start_time < 3000000)
    ;

  if (cmd == SD_INITIALIZED)
  {
    sd_init = true;
  }

  // Setting to drive,brake motors output mode
  pinMode(DRIVE_PIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);

  brake_ssr(); // Stop driving motors from any residual bootloader code

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

  rp2040.fifo.push(PLAY_DOOR_CLOSE);

  while (audio_played == 0 && sd_init)
    ;

  door_motor(DOOR_STOP, 0); // Stop closing door

  // Initialize servos to default position
  prop_servo.writeMicroseconds(450);
  prop_servo.attach(PROP_SERVO_PWM, 400, 2600);
  prop_servo.writeMicroseconds(450);
  brak_servo.writeMicroseconds(450);
  brak_servo.attach(BRAK_SERVO_PWM, 400, 2600);
  brak_servo.writeMicroseconds(450);
  steering_servo.writeMicroseconds(1475);
  steering_servo.attach(STEERING_SERVO_PWM, 400, 2600);
  steering_servo.writeMicroseconds(1475);

  // Dump reactants before starting drive
  servo_dump(prop_servo, 2500, 1000);

  // Wait for busVolatge to surpass 9V
  while (bus_voltage < 9)
  {
    raw_bus = read_register(A219_I2C, 0x02);
    bus_voltage = (raw_bus >> 3) * 0.004;
  }

  rp2040.fifo.push(PLAY_TIME_CIRCUIT);

  while (audio_played == 1 && sd_init)
    ;

  servo_dump(brak_servo, 2500, 3000);

  start_time = time_us_32(); // Braking start time

  // Poll IMU one last time
  bno08x.getSensorEvent(&sensor_value);
  quaternion_to_euler_RV(&sensor_value.un.rotationVector, &ypr, true);

  init_yaw = ypr.yaw;
  raw_yaw = init_yaw;
  prev_yaw = init_yaw;
  yaw = init_yaw;
  yaw_diff = yaw - init_yaw;

  curr_time = (time_us_32() - start_time) / 1000000.0f; // Taken to update prev_time

  rp2040.fifo.push(STATUS_STARTED);

  drive_ssr(); // Start drive
}

/*
Description: Arduino core 0 loop subroutine.
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

  raw_bus = read_register(A219_I2C, 0x02);
  bus_voltage = (raw_bus >> 3) * 0.004;

  raw_current = (int16_t)read_register(A219_I2C, 0x04);
  current_mA = raw_current * 0.1;

  // If outside of 7-13V range of > 1A current draw then stop
  if (bus_voltage > 13 || bus_voltage < 7 || current_mA > 1000)
  {
    brake_ssr();
    rp2040.fifo.push(STATUS_ERROR);
  }

  if (turbidity >= TURB_THRESHOLD && running)
  {
    brake_ssr();                                 // Stop driving
    stop_stir(BRAK_STIR_PWM_1, BRAK_STIR_PWM_2); // Stop stirring motor

    rp2040.fifo.push(STATUS_STOPPED);

    running = false; // Set flag
  }

  if (!running)
  {
    if (audio_played == 2 && !audio_trig) // Play sound effect for time travel
    {
      rp2040.fifo.push(PLAY_TIME_TRAVEL);
      audio_trig = true;
    }
    else if (audio_played == 3 && !audio_trig) // Play sound effect for doors
    {
      busy_wait_ms(500);
      door_motor(DOOR_OPEN, 255); // Start opening door
      rp2040.fifo.push(PLAY_DOOR_OPEN);

      audio_trig = true;
    }
    else if (audio_played == 4 && !audio_trig) // Done everything, run once
    {
      door_motor(DOOR_STOP, 0); // Stop opening door
      audio_trig = true;
    }
  }
}

/*
Description: Arduino core 1 setup subroutine.
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void setup1(void)
{
  mp3.begin();    // Start speaker
  stop_speaker(); // Don't overheat speaker

  // Indicate status to be initialized
  pixel.begin();
  pixel.setBrightness(255);
  pixel.setPixelColor(0, 255, 0, 0);
  pixel.show();

  // LED strips
  strip.begin();
  strip.setBrightness(255);

  for (int i = 0; i < STRIP_LEDS; i++)
  {
    strip.setPixelColor(i, 0, 0, 0);
  }

  strip.show();

  mp3.begin(); // Initialize mp3 module

  bool configured = sd.begin(config); // Initialize the SD card without blocking in case it doesn't read

  if (configured)
  {
    rp2040.fifo.push(SD_INITIALIZED);
  }

  toc = time_us_32(); // Set last update variable
}

/*
Description: Arduino core 1 loop subroutine.
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void loop1(void)
{
  uint32_t cmd = 0;
  tic = time_us_32();

  rp2040.fifo.pop_nb(&cmd);

  switch (cmd)
  {
  case PLAY_DOOR_OPEN:
    audio_file = sd.open(door_open_sound, FILE_READ); // Open corresponding SD card mp3 file
    break;
  case PLAY_DOOR_CLOSE:
    audio_file = sd.open(door_close_sound, FILE_READ); // Open corresponding SD card mp3 file
    break;
  case PLAY_TIME_TRAVEL:
    fluxing = false;
    time_jump = true;
    audio_file = sd.open(time_travel_sound, FILE_READ); // Open corresponding SD card mp3 file
    break;
  case PLAY_TIME_CIRCUIT:
    fluxing = true;
    audio_file = sd.open(time_circuit_sound, FILE_READ); // Open corresponding SD card mp3 file
    break;
  case STATUS_ERROR:
    pixel.setPixelColor(0, 255, 0, 0); // Indicate error status
    pixel.show();
    break;
  case STATUS_STARTED:
    audio_file = sd.open(theme_music, FILE_READ); // Open corresponding SD card mp3 file
    pixel.setPixelColor(0, 0, 0, 255);            // Indicate setup complete status
    pixel.show();
    break;
  case STATUS_STOPPED:
    audio_file.close();
    stop_speaker();
    pixel.setPixelColor(0, 0, 255, 0); // Indicate status to be finished
    pixel.show();
    break;
  default:
    break;
  }

  if (audio_file) // If the audio file opened successfully
  {
    send_audio(); // If the audio file has been opened and is still open, send an audio data chunk
  }

  if (fluxing)
  {
    flux_cap();
  }

  if (audio_played == 2 && !running && tic - toc > 25000)
  {
    toc = time_us_32();
    car_lights();
  }
  else if (!running && time_jump && audio_played != 2)
  {
    time_jump = false;

    for (int i = 0; i < STRIP_LEDS; i++)
    {
      strip.setPixelColor(i, 0, 0, 0);
    }

    strip.show();
  }
}
