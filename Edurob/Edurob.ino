/**************************************************************************
 * Edurob Arduino Project
 *
 * This project controls a robotic platform using ESP32-based hardware.
 * It facilitates motor control, encoder feedback, PID speed regulation,
 * kinematics calculations, and ROS2 communication for velocity commands
 * and transform publishing.
 *
 * Hardware components:
 * - Motors with PWM control
 * - Encoders for wheel speed and direction feedback
 * - ESP32 microcontroller with FreeRTOS
 *
 * Software components:
 * - PID controllers for motor speed regulation
 * - Kinematics matrices to translate robot motions to wheel speeds
 * - ROS2 micro-ROS support for command and sensor messaging
 *
 **************************************************************************/

// Project specific parameters
#include "parameter.h"

#include <WiFi.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "math.h" 

// Hardware and control libraries for PWM, Encoder, PID control, and matrix math
#include <dcpwm.h>       // PWM
#include <ESP_Counter.h> // Encoder
#include <AutoPID.h>     // PID Library https://github.com/r-downing/AutoPID
#include <Eigen.h>       // Linear math
#include <Eigen/QR>      // Calls inverse, determinant, LU decomp., etc.
using namespace Eigen;   // Simplify matrix/vector declarations

// ROS
#ifdef ROS_EN
// Include micro-ROS headers for communication and message definitions
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>

// ROS Messages
#include <geometry_msgs/msg/twist.h>
#include <tf2_msgs/msg/tf_message.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <sensor_msgs/msg/imu.h>
#include "gyroscope.h"

// Error checking macros for ROS functions
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }

#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop_soft();         \
    }                            \
  }

// ROS communication variables
rcl_subscription_t subscriber;
rcl_publisher_t tf_pub;
rcl_publisher_t gyro_pub;
geometry_msgs__msg__Twist msg;
geometry_msgs__msg__TransformStamped tfMessages[2]; // Transform between base_link / odom and base_link / laser
tf2_msgs__msg__TFMessage tf2Message;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
rclc_support_t support;
rcl_node_t node;

rcl_timer_t timer;
unsigned int timer_timeout1;
geometry_msgs__msg__Quaternion q;
sensor_msgs__msg__Imu gyro_imu_msg;

HardwareSerial ROSSerial(0);

bool serial_transport_open(struct uxrCustomTransport * transport)
{
  ROSSerial.begin(115200, SERIAL_8N1, 44, 43);
  return true;
}

bool serial_transport_close(struct uxrCustomTransport * transport)
{
  ROSSerial.end();
  return true;
}

size_t serial_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
  (void)errcode;
  size_t sent = ROSSerial.write(buf, len);
  return sent;
}

size_t serial_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
  (void)errcode;
  ROSSerial.setTimeout(timeout);
  return ROSSerial.readBytes((char *)buf, len);
}
#endif // ROS_EN

// kinematik header
#include "kinematik.h"

// Hardware-related global objects
static ESP_Counter WheelEncoder[NumMotors];      // Encoder hardware interfaces
static DCPWM MotorPWM[NumMotors];                // PWM hardware interfaces
static AutoPID *speedController[NumMotors];      // PID controller instances
static pidParam speedControllerParam[NumMotors]; // PID parameters per motor

// Encoder and filtering variables
int64_t encoderOld[NumMotors];       // Previous encoder counts for velocity calculation
int64_t encoderNew[NumMotors];       // Current encoder counts
const int windowSize = 5;            // Size for moving average filter
float inputs[NumMotors][windowSize]; // Recent velocity samples per motor
float inputsum[NumMotors];           // Summed filter inputs for average calculation
int windowIndex = 0;                 // Circular index for moving average buffer

// Setpoint and kinematics variables
static double setpointSpeed[NumMotors]; // Speed setpoints per motor
MatrixXd kinematik(4, 3);    // Forward kinematics matrix
MatrixXd kinematikInv(3, 4); // Inverse kinematics matrix
Vector3d robotSpeedSetpoint; // Desired robot speed [m/s rad/s]
Vector3d robotSpeed;         // Current robot speed
Vector3d robotSpeedMax;      // Max allowed robot speed
Vector3d robotSpeedAcc;      // Robot speed acceleration limits
Vector4d wheelSpeedSetpoint; // Desired wheel speeds [rad/s]
Vector3d robotOdom;          // Odometry position (x, y, theta)
Vector3d robotGyroOdom;          // Odometry position (x, y, theta)
Vector4d robotWheelSpeed;    // Current wheel velocities
double rosQuaternion[4];     // Quaternion for ROS transform messages


/**************************** Function Definitions ****************************/

// ROS
#ifdef ROS_EN

/**
 * @brief ROS subscriber callback to update velocity setpoints from command messages.
 * 
 * @param msgin Pointer to the incoming Twist message.
 */
void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // Update robot speed setpoints for linear and angular velocities
  robotSpeedSetpoint << msg->linear.x, msg->linear.y, msg->angular.z;
}

/**
 * @brief ROS task that publishes TF transforms to ROS in a loop.
 * 
 * This task publishes odometry and laser transforms periodically.
 * 
 * @param pvParameters FreeRTOS task parameters (unused)
 */
void ros_task(void *pvParameters)
{

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Convert yaw (theta) from robot odometry into a quaternion
    double quat[4], quatGyro[4];
    euler_to_quat(0, 0, robotOdom[2], quat);
    //euler_to_quat(0, 0, robotGyroOdom[2], quatGyro);

    // Setup odom->base_link transform
    tfMessages[0].child_frame_id.data = "base_link";
    tfMessages[0].transform.translation.x = robotOdom[0];
    tfMessages[0].transform.translation.y = robotOdom[1];
    tfMessages[0].transform.translation.z = 0;
    tfMessages[0].transform.rotation.x = quat[1];
    tfMessages[0].transform.rotation.y = quat[2];
    tfMessages[0].transform.rotation.z = quat[3];
    tfMessages[0].transform.rotation.w = quat[0];
    tfMessages[0].header.frame_id.data = "odom";
    tfMessages[0].header.stamp.sec = rmw_uros_epoch_millis() / 1000;
    tfMessages[0].header.stamp.nanosec = rmw_uros_epoch_nanos();

    // Setup base_link->laser transform
    tfMessages[1].child_frame_id.data = "laser";
    tfMessages[1].transform.translation.x = 0;
    tfMessages[1].transform.translation.y = 0;
    tfMessages[1].transform.translation.z = 0.08;
    tfMessages[1].transform.rotation.x = 0;
    tfMessages[1].transform.rotation.y = 0;
    tfMessages[1].transform.rotation.z = 0;
    tfMessages[1].transform.rotation.w = 1;
    tfMessages[1].header.frame_id.data = "base_link";
    tfMessages[1].header.stamp.sec = rmw_uros_epoch_millis() / 1000;
    tfMessages[1].header.stamp.nanosec = rmw_uros_epoch_nanos();

    // Setup odom->base_link transform
    /*
    tfMessages[2].child_frame_id.data = "base_link";
    tfMessages[2].transform.translation.x = robotGyroOdom[0];
    tfMessages[2].transform.translation.y = robotGyroOdom[1];
    tfMessages[2].transform.translation.z = 0;
    tfMessages[2].transform.rotation.x = quatGyro[1];
    tfMessages[2].transform.rotation.y = quatGyro[2];
    tfMessages[2].transform.rotation.z = quatGyro[3];
    tfMessages[2].transform.rotation.w = quatGyro[0];
    tfMessages[2].header.frame_id.data = "gyroOdom";
    tfMessages[2].header.stamp.sec = rmw_uros_epoch_millis() / 1000;
    tfMessages[2].header.stamp.nanosec = rmw_uros_epoch_nanos();
    */

    // Configure TF message container
    tf2Message.transforms.capacity = 2;
    tf2Message.transforms.data = tfMessages;
    tf2Message.transforms.size = 2;

    // Publish TF message to ROS, handle errors softly
    RCSOFTCHECK(rcl_publish(&tf_pub, &tf2Message, NULL));
  }
}

/**
 *
 *
 *
*/
  void read_imu_data  (sensor_msgs__msg__Imu &gyro_imu_msg){
    float g_x = 0.0, g_y = 0.0, g_z= 0.0;
    float a_x = 0.0, a_y= 0.0, a_z= 0.0;
    float angle_x = 0.0, angle_y = 0.0, angle_z = 0.0;
    double quat[4] = {0.0, 0.0, 0.0, 0.0};

    fetch_gyro_data(g_x, g_y, g_z);
    fetch_acceleration_data(a_x, a_y, a_z);

    quartenion_algorithm(angle_x, angle_y, angle_z);
    euler_to_quat(angle_x, angle_y, angle_z, quat);

    gyro_imu_msg.orientation.x = quat[1];
    gyro_imu_msg.orientation.y = quat[2];
    gyro_imu_msg.orientation.z = quat[3];
    gyro_imu_msg.orientation.w = quat[0];

    gyro_imu_msg.angular_velocity.x = g_x*CONVERS_CONST_DEGREES_PER_SECOND_2_RADIANS_PER_SECOND;
    gyro_imu_msg.angular_velocity.y = g_y*CONVERS_CONST_DEGREES_PER_SECOND_2_RADIANS_PER_SECOND;
    gyro_imu_msg.angular_velocity.z = g_z*CONVERS_CONST_DEGREES_PER_SECOND_2_RADIANS_PER_SECOND;
    
    gyro_imu_msg.linear_acceleration.x = a_x*CONVERS_CONST_G_2_A;
    gyro_imu_msg.linear_acceleration.y = a_y*CONVERS_CONST_G_2_A;
    gyro_imu_msg.linear_acceleration.z = a_z*CONVERS_CONST_G_2_A;

    gyro_imu_msg.header.frame_id.data = "imu_link";
    gyro_imu_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
    gyro_imu_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
  }

/**
 * @brief ROS task that publishes IMU Sensor Messages to ROS in a loop.
 *
 * This task publishes angular velocity and linear acceleration periodically. 
 *
 * @param pvParameters FreeRTOS task parameters (unused)
*/
void ros_publish_gyro_task(void *pvParameters){
  sensor_msgs__msg__Imu__init(&gyro_imu_msg);

  while(1){
    vTaskDelay(pdMS_TO_TICKS(10));

    read_imu_data(gyro_imu_msg);

    RCSOFTCHECK(rcl_publish(&gyro_pub, &gyro_imu_msg, NULL));
  }
}


#endif // ROS_EN

/**
 * @brief Infinite error loop triggered on fatal ROS errors.
 * 
 * Prints "FATAL ROS ERROR ABORTING" to serial console continuously.
 */
void error_loop()
{
  while (1)
  {
    Serial.println("FATAL ROS ERROR ABORTING");
    delay(1000);
  }
}

/**
 * @brief Soft error handler prints "SOFT ROS ERR" once.
 */
void error_loop_soft()
{
  Serial.println("SOFT ROS ERR");
}

#ifdef ROS_EN
/**
 * @brief Converts Roll-Pitch-Yaw Euler angles to a ROS quaternion message.
 * 
 * @param roll Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @param yaw Yaw angle in radians.
 * @return geometry_msgs__msg__Quaternion Constructed quaternion message.
 */
static geometry_msgs__msg__Quaternion createQauternionFromRPY(double roll, double pitch, double yaw)
{
  geometry_msgs__msg__Quaternion q;
  double halfYaw = yaw * 0.5;
  double halfPitch = pitch * 0.5;
  double halfRoll = roll * 0.5;
  double cosYaw = cos(halfYaw);
  double sinYaw = sin(halfYaw);
  double cosPitch = cos(halfPitch);
  double sinPitch = sin(halfPitch);
  double cosRoll = cos(halfRoll);
  double sinRoll = sin(halfRoll);
  q.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw; // x
  q.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw; // y
  q.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw; // z
  q.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw; // formerly yzx
  return q;
}
#endif // ROS_EN

/**
 * @brief Converts Euler angles (degrees) to a quaternion array.
 * 
 * @param x Roll angle [degrees]
 * @param y Pitch angle [degrees]
 * @param z Yaw angle [degrees]
 * @param q Output array to store quaternion [w, x, y, z]
 */
const void euler_to_quat(float x, float y, float z, double *q)
{
  float c1 = cos(y / 2.0);
  float c2 = cos(z / 2.0);
  float c3 = cos(x / 2.0);

  float s1 = sin(y / 2.0);
  float s2 = sin(z / 2.0);
  float s3 = sin(x / 2.0);

  q[0] = c1 * c2 * c3 - s1 * s2 * s3; // w
  q[1] = s1 * s2 * c3 + c1 * c2 * s3; // x
  q[2] = s1 * c2 * c3 + c1 * s2 * s3; // y
  q[3] = c1 * s2 * c3 - s1 * c2 * s3; // z
}

/**
 * @brief Checks direction of rotation of encoder.
 * 
 * Samples encoder counts twice with delay and determines if count is increasing
 * or decreasing, indicating direction.
 * 
 * @param encoder Reference to encoder hardware interface (ESP_Counter).
 * @return int 1 = forward (count increased), -1 = reverse (count decreased), 0 = no change.
 */
int checkEncoderDirection(ESP_Counter &encoder)
{
  int count = encoder.getCount();
  delay(50);
  int count2 = encoder.getCount();
  if (count2 > count)
  {
    return 1;
  }
  else if (count2 < count)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

/**
 * @brief Rotates a 2D vector by angle r (radians).
 * 
 * Applies rotation matrix multiplication to update coordinates in place.
 * 
 * @param r Rotation angle in radians
 * @param x Reference to x component, updated in place
 * @param y Reference to y component, updated in place
 */
const void rotate2D(float r, double &x, double &y)
{
  double temp = x * cos(r) - y * sin(r);
  y = x * sin(r) + y * cos(r);
  x = temp;
}

/**
 * @brief Prints contents of an Eigen matrix to the serial console.
 * 
 * Useful for debug logging matrix values step-by-step.
 * 
 * @param X MatrixXd reference to print.
 */
void print_mtxd(const Eigen::MatrixXd &X)
{
  int i, j, nrow, ncol;

  nrow = X.rows();
  ncol = X.cols();

  Serial.print("nrow: ");
  Serial.println(nrow);
  Serial.print("ncol: ");
  Serial.println(ncol);
  Serial.println();

  for (i = 0; i < nrow; i++)
  {
    for (j = 0; j < ncol; j++)
    {
      Serial.print(X(i, j), 6); // print 6 decimal places
      Serial.print(", ");
    }
    Serial.println();
  }
  Serial.println();
}

/**
 * @brief Initialize hardware PWM and encoder peripherals.
 * 
 * Configures pins, initializes encoders and PWM units for motors,
 * determines encoder direction, enables motors and resets counters.
 * 
 * @return true if successful, false if any initialization fails.
 */
bool initHardware()
{
  pinMode(EnablePIN, OUTPUT);   // Motor enable signal
  digitalWrite(EnablePIN, LOW); // Disable motors while initializing
  for (int i = 0; i < NumMotors; i++)
  {
    if (!WheelEncoder[i].init(i, Enc_A[i], Enc_B[i]))
    {
      return false;
    }
  }

  int currentChannel = 0;
  for (int i = 0; i < NumMotors; i++)
  {
    if (!MotorPWM[i].init(30000, 10, currentChannel, currentChannel + 1, PWM_A[i], PWM_B[i]))
    {
      return false;
    }
    MotorPWM[i].setPWM(0); // Set initial speed to 0
    currentChannel += 2;
  }
  digitalWrite(EnablePIN, HIGH); // Enable motors
  for (int i = 0; i < NumMotors; i++)
  {
    MotorPWM[i].setPWM(100 * MotorDir[i]); // Briefly run to confirm encoder direction
    EncoderDir[i] = checkEncoderDirection(WheelEncoder[i]);
    MotorPWM[i].setPWM(0);
  }
  delay(1000); // Allow counts to stabilize
  for (int i = 0; i < NumMotors; i++)
  {
    WheelEncoder[i].resetCounter();
  }

  return true;
}

/**
 * @brief Initialize PID speed controllers with default parameters.
 * 
 * Sets PID gains, sampling time, output limits for each motor's speed control.
 */
void initPID()
{
  for (int i = 0; i < NumMotors; i++)
  {
    speedControllerParam[i].input = 0;
    speedControllerParam[i].output = 0;
    speedControllerParam[i].setpoint = 0;
    speedControllerParam[i].p = 0.04;
    speedControllerParam[i].i = 0.08;
    speedControllerParam[i].d = 0.0;
    speedControllerParam[i].sampleTimeMs = sampleTime;
    speedControllerParam[i].outMin = -100;
    speedControllerParam[i].outMax = 100;

    speedController[i] = new AutoPID(&speedControllerParam[i].input, &speedControllerParam[i].setpoint, &speedControllerParam[i].output, speedControllerParam[i].outMin, speedControllerParam[i].outMax, speedControllerParam[i].p, speedControllerParam[i].i, speedControllerParam[i].d);
    speedController[i]->setTimeStep(speedControllerParam[i].sampleTimeMs);
    speedController[i]->setBangBang(0, 0); // Disable BangBang-Controll
  }
}

/**
 * @brief Initialize kinematic matrices depending on robot type.
 * 
 * Sets forward and inverse kinematics matrices for robot motion translation.
 * Sets initial speed vectors and acceleration/speed limits.
 */
void initMatrix()
{

#ifdef MECANUM
  // Mecanum
  mecanum_matrix(kinematik, kinematikInv, l1, l2); // sets the kinematik and kinematikInv to the desired values
#endif

#ifdef DIFF
  // Diff
  differential_matrix(kinematik, kinematikInv, l1, l2); // sets the kinematik and kinematikInv to the desired values

#endif

#ifdef OMNI4
  // Omni 4 Wheels
  omni_4_matrix(kinematik, kinematikInv, l1, l2); // sets the kinematik and kinematikInv to the desired values
#endif

#ifdef OMNI3
  // Omni 3 Wheels
  omni_3_matrix(kinematik, kinematikInv, l1, l2); // sets the kinematik and kinematikInv to the desired values
#endif

  robotSpeedSetpoint << 0.0,
      0.0,
      0.0;

  robotSpeedAcc << maxTAccel, maxTAccel, maxRAccel;
  robotSpeedMax << maxTSpeed, maxTSpeed, maxRSpeed;
  wheelSpeedSetpoint = (1 / wheelRadius) * kinematik * robotSpeedSetpoint;
}

/**
 * @brief FreeRTOS task that runs the speed controller loop.
 * 
 * Applies PID control for motor speeds using encoder feedback and setpoints.
 * Updates robot odometry estimates.
 * 
 * @param pvParameters Task parameters (unused)
 */
void speedControllerTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(sampleTime);
  Vector4d EncoderRadDiff;
  Vector3d robotOdomChange;
  Vector3d worldOdomChange;
  Vector3d worldGyroOdomChange;

  float old_rotation_degree_z = 0.0f;

  for (int i = 0; i < NumMotors; i++)
  {
    inputsum[i] = 0;
  }

  for (int i = 0; i < NumMotors; i++)
  {
    for (int k = 0; k < windowSize; k++)
    {
      inputs[i][k] = 0;
    }
  }

  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Adjust current robot speeds to setpoints respecting velocity limits
    for (int i = 0; i < 3; i++)
    {
      robotSpeed[i] = robotSpeedSetpoint[i];

      // Clamp to max speeds
      if (robotSpeed[i] > robotSpeedMax[i])
      {
        robotSpeed[i] = robotSpeedMax[i];
      }
      else if (robotSpeed[i] < -robotSpeedMax[i])
      {
        robotSpeed[i] = -robotSpeedMax[i];
      }

      // Zero small speeds for stability
      if (robotSpeed[i] < 0.000001 && robotSpeed[i] > -0.000001)
      {
        robotSpeed[i] = 0;
      }
    }

    // Calculate wheel speed setpoints with kinematics matrix
    wheelSpeedSetpoint = (1 / wheelRadius) * kinematik * robotSpeed;

    // Deadzone elimination for minimal wheel speeds
    for (int i = 0; i < NumMotors; i++)
    {
      if (abs(wheelSpeedSetpoint[i]) < 0.15)
      {
        wheelSpeedSetpoint[i] = 0;
      }
    }

    // Set PID controller setpoints for each motor
    for (int i = 0; i < NumMotors; i++)
    {
      speedControllerParam[i].setpoint = wheelSpeedSetpoint(i, 0);
    }

    // Update encoder readings and filtered speed inputs
    for (int i = 0; i < NumMotors; i++)
    {
      encoderNew[i] = EncoderDir[i] * WheelEncoder[i].getCount();

      // Get the increments since last run
      EncoderRadDiff[i] = (encoderNew[i] - encoderOld[i]) * incrementsToRad;

      inputsum[i] -= inputs[i][windowIndex];
      inputs[i][windowIndex] = ((((encoderNew[i] - encoderOld[i]) * incrementsToRad) / sampleTime) * 1000);
      inputsum[i] += inputs[i][windowIndex];

      speedControllerParam[i].input = inputsum[i] / windowSize; // Moving average filter

      speedController[i]->run();

      MotorPWM[i].setPWM(MotorDir[i] * speedControllerParam[i].output * Rad2PWM); // Raw
      encoderOld[i] = encoderNew[i];
    }

    // Update wheel speed vector for odometry calculations
    for (int i = 0; i < NumMotors; i++)
    {
      robotWheelSpeed[i] = inputs[i][windowIndex];
    }

    // Calculate the robot's state change
    robotOdomChange = (wheelRadius * kinematikInv * EncoderRadDiff);
    worldOdomChange = robot_vel_to_world_vel(robotOdom[2] + robotOdomChange[2]/2.0, robotOdomChange);
    worldGyroOdomChange = robot_vel_to_world_vel(robotGyroOdom[2] + 1.0/2.0, worldGyroOdomChange);
    old_rotation_degree_z = 0;
   
    // Update odometry position
    robotOdom += worldOdomChange;
    robotGyroOdom += worldGyroOdomChange;    

    windowIndex = (windowIndex + 1) % windowSize;
  }
}

/**
 * @brief Logger task periodically outputs odometry data over serial.
 * 
 * @param pvParameters Task parameters (unused)
 */
void loggerTask(void *pvParameters)
{
  while (true)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print("Odometry: Xt= ");
    Serial.print(robotOdom[0]); // current x-position in meters
    Serial.print(", Yt= ");
    Serial.print(robotOdom[1]); // current y-position in meters
    Serial.print(", Zr= ");
    Serial.println(robotOdom[2]); // current z-rotation in radians
  }
}

/**
 * @brief Arduino setup function, runs once at startup.
 * 
 * Initializes serial, hardware, PID, kinematics, and creates FreeRTOS tasks.
 * Also initializes micro-ROS resources if enabled.
 */
void setup()
{
  Serial.begin(115200);
  initHardware();
  initPID();
  initMatrix();
  init_IMU();

  delay(2000);

#ifdef ROS_EN 
  //set_microros_wifi_transports(WIFISSID, WIFIPASS, AGENT_IP, AGENT_PORT);
  rmw_uros_set_custom_transport(true, NULL, serial_transport_open, serial_transport_close, serial_transport_write, serial_transport_read);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "EduRob_Node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(&tf_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "tf"));
  RCCHECK(rclc_publisher_init_default(&gyro_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "gyro"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
#endif // ROS_EN

  xTaskCreatePinnedToCore(
      speedControllerTask,   /* Task function. */
      "speedControllerTask", /* String with name of task. */
      10000,                 /* Stack size in bytes. */
      NULL,                  /* Parameter passed as input of the task */
      5,                     /* Priority of the task. */
      NULL, 1);              /* Task handle. */ 
#ifdef ROS_EN
  xTaskCreatePinnedToCore(
      ros_task,   /* Task function. */
      "ros_task", /* String with name of task. */
      10000,      /* Stack size in bytes. */
      NULL,       /* Parameter passed as input of the task */
      4,          /* Priority of the task. */
      NULL, 1);   /* Task handle. */
  xTaskCreatePinnedToCore(
      ros_publish_gyro_task,   /* Task function. */
      "ros_publish_gyro_task", /* String with name of task. */
      10000,      /* Stack size in bytes. */
      NULL,       /* Parameter passed as input of the task */
      3,          /* Priority of the task. */
      NULL, 1);   /* Task handle. */  
#endif // ROS_EN
  xTaskCreatePinnedToCore(
      loggerTask,   /* Task function. */
      "loggerTask", /* String with name of task. */
      20000,        /* Stack size in bytes. */
      NULL,         /* Parameter passed as input of the task */
      1,            /* Priority of the task. */
      NULL, 1);     /* Task handle. */
}

/**
 * @brief Arduino main loop, runs repeatedly.
 * 
 * Enables motors and processes ROS incoming commands if enabled.exit
 */
void loop()
{
  // #############-USER-CODE-START-#####################
  // double tx = 0.0, ty = 0.0, theta = 0.0;
  // robotSpeedSetpoint << tx, ty, theta;

  // #############-USER-CODE-END-#####################

#ifdef ROS_EN
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
#endif // ROS_EN

  delay(10);
}
