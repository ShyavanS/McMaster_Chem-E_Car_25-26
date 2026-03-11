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
#include <OneWire.h>
#include <Adafruit_BNO08x.h>
#include <DallasTemperature.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include "hardware/timer.h"
#include "encoder/encoder.cpp" // Modified to remove debounce delays, don't replace!
#include "encoder/encoder.hpp" // Modified to remove debounce delays, don't replace!
#include "SdFat.h"
#include <Wire.h>
#include <BackgroundAudio.h>
#include <PWMAudio.h>

#define NUM_LEDS 1 // Status LED

// Define drive motor pins
#define LEFT_PWM_1 9
#define LEFT_PWM_2 10
#define RIGHT_PWM_1 12
#define RIGHT_PWM_2 11

// Define the PWM pins for the stir bar motors
#define BRAK_STIR_PWM_1 A3
#define BRAK_STIR_PWM_2 24
#define PROP_STIR_PWM_1 6
#define PROP_STIR_PWM_2 5

// Define servo pins
#define BRAK_SERVO_PWM 13
#define PROP_SERVO_PWM 4
#define LEFT_SERVO_PWM MOSI
#define RIGHT_SERVO_PWM SCK

// Define encoder pins
#define ENC_A A0
#define ENC_B A1

#define BRAK_TEMP_SENS A1 // Pin for the teperature sensor data line

#define BNO08X_RESET -1 // No reset pin for IMU over I2C, only enabled for SPI

// Define chip select pin for SD card
#define SD_CS_PIN 23

#define A219_I2C 0x40 // I2C address for current/voltage sensor
// Define audio output pin
#define AUDIO_OUT 12

using namespace encoder;

// Struct for Euler Angles
struct euler_t
{
  float yaw;
  float pitch;
  float roll;
} ypr;

// Encoder constants
const float PPR = 8192.0;
const double WHEEL_CIRCUMFERENCE_M = 0.398982;

// Create servo objects
Servo brak_servo;
Servo prop_servo;
Servo left_servo;
Servo right_servo;

// Create encoder object using only A & B pins, not index, assign to pio0, sm 1 & 3 in reversed direction w/ microstepping for smoothness
Encoder drive_encoder(PIO pio0, 3, {ENC_A, ENC_B}, PIN_UNUSED, REVERSED_DIR, PPR, true);

// Create BNO085 instance
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensor_value;

OneWire one_wire(BRAK_TEMP_SENS);          // Create a OneWire instance to communicate with the sensor
DallasTemperature temp_sensors(&one_wire); // Pass OneWire reference to Dallas Temperature sensor

Adafruit_NeoPixel pixel(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800); // Status LED

// Define files
SdFat sd;
FsFile root;
FsFile next_file;
FsFile data_file;
FsFile audio_file;
String start_music = "start_music.mp3"; // placeholders for actual audio file names
String stop_music = "stop_music.mp3";
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);
String file_name;

// Set up audio output using PWM
PWMAudio pwm(AUDIO_OUT);
BackgroundAudioMP3 mp3(pwm); // create the mp3 player object and decoder
uint8_t filebuff[512]; // allocates 512 bytes of memory to temporarily store audio data before processing

// Flag to check if audio started playing
bool audio_started = false;


bool is_file_new = true; // Checks for new file

// The target yaw angle to keep car straight
const float GOAL_YAW = 0.0;

// Rejection threshold for noise in IMU measurements
const float REJECT_THRESHOLD = 3.0;

// Define IMU variables
double raw_yaw;        // raw yaw angle
double prev_yaw;       // previous unwrapped yaw angle
double yaw;            // unwrapped yaw angle
double init_yaw = 0.0; // initial yaw angle
double yaw_diff = 0.0; // yaw angle difference

// Wheel distance variables
double drive_dist_m = 0.0;

// Delta temperature
double temp_diff;

// Temperature change threshold
double temp_change;

// Last temperature fetched flag
bool last_fetch;

// Initialize run count for SD card file
int run_count;

// variables to store temperature
double temperature_c; // Current temperature
double init_temp;     // Initial temperature for differential calculation

// KALMAN FILTER variables
double x_temp; // Filtered temperature
double p_temp; // Initial error covariance
double x_imu;  // Filtered temperature
double p_imu;  // Initial error covariance

// Process noise and measurement noise
double q_temp; // Process noise covariance
double r_temp; // Measurement noise covariance
double q_imu;  // Process noise covariance
double r_imu;  // Measurement noise covariance

// Keeping track of time
double curr_time = 0.0f;
double prev_time = 0.0f;
uint32_t start_time;

const int DATA_SIZE = 11; // Number of items to log
const int DATA_SIZE = 8; // Number of items to log
double data[DATA_SIZE];  // Data array

// PID loop variables
double error = 0.0;      // Proportional error
double last_error = 0.0; // Derivative error
double sum_error = 0.0;  // Integral error

// The following numbers need to be adjusted through testing
const float K_P = 13.0; // Proportional weighting
const float K_I = 0.0;  // Integral weighting
const float K_D = 0.0;  // Derivative weighting

// Offsets & constants for PID
int left_offset = 0;
int right_offset = 0;
const int SERVO_ANGLE = 1475;
const int MAX_OFFSET = 1024;
const int YAW_REF = 90;

/*
Description: Subroutine to drive car forward.
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void drive_forward(void)
{
  digitalWrite(LEFT_PWM_1, HIGH);
  digitalWrite(RIGHT_PWM_2, HIGH);
  digitalWrite(LEFT_PWM_2, LOW);
  digitalWrite(RIGHT_PWM_1, LOW);
}

/*
Description: Subroutine to stop car and brake motors.
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void stop_driving(void)
{
  digitalWrite(LEFT_PWM_1, HIGH);
  digitalWrite(RIGHT_PWM_2, HIGH);
  digitalWrite(LEFT_PWM_2, HIGH);
  digitalWrite(RIGHT_PWM_1, HIGH);
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
  digitalWrite(stir_pin_1, LOW);  // For fast decay
  analogWrite(stir_pin_2, speed); // Set motor to speed obtained through testing
}

/*
Description: Subroutine to implement Kalman filtering on temperature sensor data.
Inputs:      void
Outputs:     (double)x_temp, (double)p_temp
Parameters:  (double)x_k, (double)p_k, (double)q, (double)r, (double)input
Returns:     void
*/
void kalman_filter(double x_k, double p_k, double q, double r, double input, bool tempTrue)
{
  // Kalman filter prediction
  double x_k_minus = x_k;     // Predicted next state estimate
  double p_k_minus = p_k + q; // Predicted error covariance for the next state

  // Kalman filter update

  /* Kalman gain: calculated based on the predicted error covariance
  and the measurement noise covariance, used to update the
  state estimate (x_k) and error covariance (p_k) */
  double k = p_k_minus / (p_k_minus + r); // Kalman gain

  // Comparison with actual sensor reading
  x_k = x_k_minus + k * (input - x_k_minus); // Updated state estimate
  p_k = (1 - k) * p_k_minus;                 // Updated error covariance

  if (tempTrue) // Update state for temperature sensor or IMU accordingly
  {
    x_temp = x_k;
    p_temp = p_k;
  }
  else
  {
    x_imu = x_k;
    p_imu = p_k;
  }
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

  kalman_filter(x_imu, p_imu, q_imu, r_imu, yaw_diff, false); // Kalman filtering for IMU data

  // Update errors
  last_error = error;
  error = GOAL_YAW - x_imu;
  sum_error = max(min(sum_error + cbrt(error), MAX_OFFSET), -MAX_OFFSET);

  // Write to servos
  left_servo.writeMicroseconds(SERVO_ANGLE - adj_pid_output);
  right_servo.writeMicroseconds(SERVO_ANGLE - adj_pid_output);
}

/*
Description: Subroutine to fetch temperature form the temperature sensor when the data is ready.
Inputs:      void
Outputs:     (double)temperature_c
Parameters:  void
Returns:     void
*/
void fetch_temp(void)
{
  temperature_c = temp_sensors.getTempCByIndex(0); // Get temperature in Celsius

  // Update temperature kalman filter
  kalman_filter(x_temp, p_temp, q_temp, r_temp, temperature_c, true);

  temp_diff = x_temp - init_temp; // Update delta temperature
  last_fetch = true;              // Raise fetch flag to signal ready
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
Description: Send audio data chunks to the decoder and close file when done
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void send_audio(void)
{
  // If an audio file is open and the decoder buffer has enough space for more audio data
  while (audio_file && mp3.availableForWrite() > 512) 
  {
    // read 512 bytes from the audio file and send to the decoder
    int len = audio_file.read(filebuff, 512);
    mp3.write(filebuff, len);

    if (len!=512) // if the audio data was shorter than 512 bytes, we've reached the end of the file
    {
      audio_file.close();
      audio_started = false;
    }
  }
}

/*
Description: Helper function to write value to register over I2C
Inputs: void
Outputs: void
Parameters: address (unsigned 8-bit), reg (unsigned 8-bit), value (unsigned 16-bit)
Returns: void
*/
void writeRegister(uint8_t address, uint8_t reg, uint16_t value) {
  Wire.beginTransmission(address);

  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF); // MSB
  Wire.write(value & 0xFF); // LSB

  Wire.endTransmission();
}

/*
Description: Helper function to read value from register over I2C
Inputs: void
Outputs: void
Parameters: address (unsigned 8-bit), reg (unsigned 8-bit)
Returns: 2 bytes of data
*/
uint16_t readRegister(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)2); // Request 2 bytes
  if(Wire.available() >= 2) { // If data available then return it
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
  writeRegister(INA219_ADDR, 0x00, 0x15DF);

  /*
  Write to calibration register (0x05)
  Assumes 0.1 Ohm resistor and 2.0A max current (0.1 mA per bit)
  Uses formula 0.04096/(Resistor*Current_Per_Bit)
  Writes 4096 in this case
  */
  writeRegister(A219_I2C, 0x05, 0x1000);

  // --- READ BUS VOLTAGE (0x02) ---
  uint16_t rawBus = readRegister(A219_I2C, 0x02);
  // Shift right 3 bits to remove status flags, then multiply by 4mV
  float busVoltage = (rawBus >> 3) * 0.004;

  // --- READ CURRENT (0x04) ---
  int16_t rawCurrent = (int16_t)readRegister(A219_I2C, 0x04);
  // Multiply by calculated LSB (0.1mA)
  float current_mA = rawCurrent * 0.1;

  // Wait for busVolatge to surpass 7V
  while(busVoltage < 7) {
    rawBus = readRegister(A219_I2C, 0x02);
    busVoltage = (rawBus >> 3) * 0.004;
  }

  // Indicate status to be initialized
  pixel.begin();
  pixel.setBrightness(255);
  pixel.show();
  pixel.setPixelColor(0, 255, 0, 0);
  pixel.show();

  Serial.begin(115200);

  while (!sd.begin(config))
  {
    busy_wait_ms(1000); // Wait for a second before retrying
  }

  root = sd.open("/", FILE_READ); // Open SD root directory
  run_count = 0;

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

  // Setting to drive motors output mode
  pinMode(LEFT_PWM_1, OUTPUT);
  pinMode(LEFT_PWM_2, OUTPUT);
  pinMode(RIGHT_PWM_1, OUTPUT);
  pinMode(RIGHT_PWM_2, OUTPUT);

  stop_driving(); // Stop driving motors from any residual bootloader code

  // Initialize the stir motor pins as outputs
  pinMode(BRAK_STIR_PWM_1, OUTPUT);
  pinMode(BRAK_STIR_PWM_2, OUTPUT);
  pinMode(PROP_STIR_PWM_1, OUTPUT);
  pinMode(PROP_STIR_PWM_2, OUTPUT);

  // Setting the stir speed
  start_stir(BRAK_STIR_PWM_1, BRAK_STIR_PWM_2, 255);
  start_stir(PROP_STIR_PWM_1, PROP_STIR_PWM_2, 255);

  temp_sensors.begin();                     // Initialize the DS18B20 sensor
  temp_sensors.setResolution(11);           // Reduce resolution for faster polling
  temp_sensors.requestTemperatures();       // Request temperature from all devices on the bus
  temp_sensors.setWaitForConversion(false); // Disable blocking to allow multitasking

  init_temp = temp_sensors.getTempCByIndex(0); // Get temperature in Celsius
  temperature_c = init_temp;                   // Initialize temperature variable
  last_fetch = true;                           // Raise fetch flag to signal ready
  temp_diff = 0.0;                             // Initialize delta temperature to zero

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

  // Initialize Kalman filter parameters
  x_temp = init_temp; // Initial state estimate
  p_temp = 0.1;       // Initial error covariance
  q_temp = 0.01;      // Process noise covariance
  r_temp = 0.5;       // Measurement noise covariance
  x_imu = yaw_diff;   // Initial state estimate
  p_imu = 0.007;      // Initial error covariance
  q_imu = 0.005;      // Process noise covariance
  r_imu = 0.01;       // Measurement noise covariance

  // Initialize servos to default position
  prop_servo.writeMicroseconds(450);
  prop_servo.attach(PROP_SERVO_PWM, 400, 2600);
  prop_servo.writeMicroseconds(450);
  busy_wait_ms(2000);
  brak_servo.writeMicroseconds(450);
  brak_servo.attach(BRAK_SERVO_PWM, 400, 2600);
  brak_servo.writeMicroseconds(450);
  busy_wait_ms(2000);
  left_servo.writeMicroseconds(1475);
  left_servo.attach(LEFT_SERVO_PWM, 400, 2600);
  left_servo.writeMicroseconds(1475);
  busy_wait_ms(2000);
  right_servo.writeMicroseconds(1475);
  right_servo.attach(RIGHT_SERVO_PWM, 400, 2600);
  right_servo.writeMicroseconds(1475);
  busy_wait_ms(2000);

  // Dump reactants before starting drive
  servo_dump(prop_servo, 2500, 3000);
  busy_wait_ms(7000);
  servo_dump(brak_servo, 2500, 3000);

  start_time = time_us_32(); // First measurement saved seperately

  busy_wait_ms(17000);

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

  mp3.begin();
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
  // Play start music
  if (!audio_started) // If the audio hasn't started playing
  {
    audio_file = sd.open(start_music, FILE_READ); // Open corresponding SD card mp3 file
    if (audio_file) // If the audio file opened successfully
    {
      audio_started = true; // Flag that the audio file has been opened
    }
  }

  if (audio_started && audio_file) // If the audio file has been opened and is still open, send an audio data chunk
  {
    send_audio();
  }
  
  drive_forward(); // Start drive

  prev_time = curr_time;

  // Only poll if flag indicates ready state
  if (last_fetch)
  {
    temp_sensors.requestTemperatures(); // Request temperature from all devices on the bus

    last_fetch = false; // System is no longer ready, lower flag
  }
  else if (temp_sensors.isConversionComplete())
  {
    fetch_temp(); // Fetch temperature after conversion, otherwise continue loop
  }

  curr_time = (time_us_32() - start_time) / 1000000.0f; // Taken to check time against first measurement

  pid_loop(); // Run PID controller

  // Convert encoder counts to distance
  drive_dist_m = (double)drive_encoder.count() / PPR * WHEEL_CIRCUMFERENCE_M;

  uint16_t rawBus = readRegister(A219_I2C, 0x02);
  double busVoltage = (rawBus >> 3) * 0.004;

  int16_t rawCurrent = (int16_t)readRegister(A219_I2C, 0x04);
  double current_mA = rawCurrent * 0.1;

  // If outside of 3-14V range of > 1A current draw then stop
  if(busVoltage > 14 || busVoltage < 3 || current_mA > 1000) {
    pixel.setPixelColor(0, 255, 0, 0); // Turn LED to red
    pixel.show();

    stop_driving();

    while (1)
      ; // Do nothing for remainder of uptime
  }

  // delay(5) If needed since sensor needs about 5ms to process 8 samples


  // Update data array
  data[0] = temperature_c;
  data[1] = x_temp;
  data[2] = temp_diff;
  data[3] = temp_change;
  data[4] = yaw;
  data[5] = yaw_diff;
  data[6] = x_imu;
  data[7] = drive_dist_m;
  data[8] = current_mA;
  data[9] = busVoltage;
  

  // Open csv file
  data_file = sd.open(file_name, FILE_WRITE);

  // Write to csv file
  if (data_file)
  {
    // Write file header
    if (is_file_new)
    {
      data_file.println("Time (s),Raw Temperature (deg C),Filtered Temperature (deg C),Delta T (deg C),Temperature Line (deg C),Raw Yaw Angle (deg),Delta Yaw Angle (deg),Filtered Yaw Angle (deg),Current (mA),Voltage (V),Wheel Distance (m)");
      is_file_new = false;
    }

    printer(false, curr_time, data); // Write variable data to the file in CSV format

    data_file.close();
  }

  printer(true, curr_time, data); // Write variable data to serial in CSV format

  temp_change = 0.185f * curr_time - 4.5f; // Calculate temperature change

  if (temp_diff <= temp_change)
  {
    // Stop driving
    stop_driving();

    // Indicate status to be finished
    pixel.setPixelColor(0, 0, 255, 0);
    pixel.show();

    // Play stop music
    audio_file = sd.open(stop_music, FILE_READ);
    while(audio_file) //keep playing audio until file is closed
    {
      send_audio();
    }

    while (1)
      ; // Do nothing for remainder of uptime
  }
}
