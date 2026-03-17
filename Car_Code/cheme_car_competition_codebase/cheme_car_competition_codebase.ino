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
#include "hardware/timer.h"

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

// #define TURBIDITY_SENS A2 // Pin for the turbidity sensor data line
#define TURBIDITY_SENS A2

#define BNO08X_RESET -1 // No reset pin for IMU over I2C, only enabled for SPI

// Struct for Euler Angles
struct euler_t
{
  float yaw;
  float pitch;
  float roll;
} ypr;

// Create servo objects
Servo brak_servo;
Servo prop_servo;
Servo left_servo;
Servo right_servo;

// Create BNO085 instance
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensor_value;


Adafruit_NeoPixel pixel(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800); // Status LED

// The target yaw angle to keep car straight
const float GOAL_YAW = 0.0;

// Rejection threshold for noise in IMU measurements
const float REJECT_THRESHOLD = 3.0;

// define turbidity threshold for braking(THIS IS A PLACEHOLDER)
const float TURB_THRESHOLD = 3000.0; // THIS IS A PLACEHOLDER!!!

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
void start_stir(int stir_pin_1, int stir_pin_2, int speed) // Start stirring mechanism
{
  digitalWrite(stir_pin_1, LOW);  // For fast decay
  analogWrite(stir_pin_2, speed); // Set motor to speed obtained through testing
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
float fetch_turb(int samples)
{
  // calculate turbidity(avg of # measurements)
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
Description: Arduino setup subroutine.
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void setup(void)
{
  // Indicate status to be initialized
  pixel.begin();
  pixel.setBrightness(255);
  pixel.show();
  pixel.setPixelColor(0, 255, 0, 0);
  pixel.show();

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

  start_time = micros(); // First measurement saved seperately

  busy_wait_ms(17000);

  // Poll IMU one last time
  bno08x.getSensorEvent(&sensor_value);
  quaternion_to_euler_RV(&sensor_value.un.rotationVector, &ypr, true);

  init_yaw = ypr.yaw;
  raw_yaw = init_yaw;
  prev_yaw = init_yaw;
  yaw = init_yaw;
  yaw_diff = yaw - init_yaw;

  curr_time = (time_us_32() - start_time) / 1000000.0f; // Taken to update prev_time

  pixel.setPixelColor(0, 0, 0, 255); // Indicate setup complete status
  pixel.show();
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
  drive_forward(); // Start drive

  prev_time = curr_time;

  
  turbidity = fetch_turb(20);//fetch turbidity measurement
  

  curr_time = (time_us_32() - start_time) / 1000000.0f; // Taken to check time against first measurement

  pid_loop(); // Run PID controller


  if (TURB_THRESHOLD <= turbidity)
  {
    // Stop driving
    stop_driving();

    // Indicate status to be finished
    pixel.setPixelColor(0, 0, 255, 0);
    pixel.show();

    while (1)
      ; // Do nothing for remainder of uptime
  }
}
