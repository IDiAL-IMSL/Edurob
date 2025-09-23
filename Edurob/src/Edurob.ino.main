#define ROS_EN

#include <WiFi.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "math.h"

#include <dcpwm.h>       // PWM
#include <ESP_Counter.h> // Encoder
#include <AutoPID.h>     // PID Library https://github.com/r-downing/AutoPID
#include <Eigen.h>       // Linear math
#include <Eigen/QR>      // Calls inverse, determinant, LU decomp., etc.
using namespace Eigen;   // Eigen related statement; simplifies syntax for declaration of matrices

// ROS
#ifdef ROS_EN
#undef ESP32
#include <ros.h>
#define ESP32
#include <stdio.h>

// ROS Messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tfMessage.h>
#endif ROS_EN

// Project specific headers
#include "parameter.h"

// kinematik header
#include "kinematik.h"

// Hardware
static ESP_Counter WheelEncoder[NumMotors];      // Hardware-Encoder-Units
static DCPWM MotorPWM[NumMotors];                // Hardware-PWM-Units
static AutoPID *speedController[NumMotors];      // PID-Units
static pidParam speedControllerParam[NumMotors]; // PID-Parameter

// Variables
int64_t encoderOld[NumMotors];       // Last encoder values
int64_t encoderNew[NumMotors];       // Current encoder values
const int windowSize = 5;            // Number of values in moving average window
float inputs[NumMotors][windowSize]; // Input values for moving average
float inputsum[NumMotors];           // Sum of values for moving average
int windowIndex = 0;                 // Currently accessed cell of inputs[motornum][x];

// Setpoints
static double setpointSpeed[NumMotors]; // Motorspeed setpoints

// Matrix
MatrixXd kinematik(4, 3);    // Kinematics matrix for differential
MatrixXd kinematikInv(3, 4); // Inverse Kinematics matrix for differential
Vector3d robotSpeedSetpoint; // Vector with translationional and rotational robot speeds in m/s and rad/s  (X(m/s), Y(m/s), Z(rad/s))
Vector3d robotSpeed;         // Vector with translationional and rotational robot speeds in m/s and rad/s  (X(m/s), Y(m/s), Z(rad/s))
Vector3d robotSpeedMax;      // Vector with translationional and rotational robot speeds in m/s and rad/s  (X(m/s), Y(m/s), Z(rad/s))
Vector3d robotSpeedAcc;      // Vector with translationional and rotational robot speeds in m/s and rad/s  (X(m/s), Y(m/s), Z(rad/s))
Vector4d wheelSpeedSetpoint; // Wheel speeds in rad/s
Vector3d robotOdom;          // Odometry Position
Vector4d robotWheelSpeed;    // Current wheel speeds
Vector3d robotOdomSpeed;     // Odometry Speed
double rosQuaternion[4];     // Quaternion for ROS transform message

// ROS
#ifdef ROS_EN

ros::NodeHandle nh;

geometry_msgs::TransformStamped tfData;
geometry_msgs::TransformStamped tfStaticData;
tf::tfMessage messageTf;
tf::tfMessage messageTfStatic;
ros::Publisher tf_pub("/tf", &messageTf);
ros::Publisher tf_static_pub("/tf_static", &messageTfStatic);
#endif // ROS_EN

// Error function in case of unhandeld ros-error
void error_loop()
{
  while (1)
  {
    Serial.println("ERR");
    delay(100);
  }
}

// Convert Eulerdegrees to quaternion
const void euler_to_quat(float x, float y, float z, double *q)
{
  float c1 = cos(y / 2.0);
  float c2 = cos(z / 2.0);
  float c3 = cos(x / 2.0);

  float s1 = sin(y / 2.0);
  float s2 = sin(z / 2.0);
  float s3 = sin(x / 2.0);

  q[0] = c1 * c2 * c3 - s1 * s2 * s3;
  q[1] = s1 * s2 * c3 + c1 * c2 * s3;
  q[2] = s1 * c2 * c3 + c1 * s2 * s3;
  q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}

// checks the direction of the encoder
int checkEncoderDirection(ESP_Counter &encoder)
{
  int count = encoder.getCount();
  delay(500);
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

// Rotate 2D-Vector
const void rotate2D(float r, double &x, double &y)
{
  double temp = x * cos(r) - y * sin(r);
  y = x * sin(r) + y * cos(r);
  x = temp;
}

#ifdef ROS_EN
// Twist message cb
void subscription_callback(const geometry_msgs::Twist &msgin)
{
  robotSpeedSetpoint << msgin.linear.x, msgin.linear.y, msgin.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &subscription_callback);
#endif ROS_EN

// Print content of Eigen::MatrixXd
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

// Init Encoder- and PWM-Units
bool initHardware()
{
  pinMode(EnablePIN, OUTPUT);   // Motor enable signal
  digitalWrite(EnablePIN, LOW); // Disable motors
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
    MotorPWM[i].setPWM(100 * MotorDir[i]); // Start Motor
    EncoderDir[i] = checkEncoderDirection(WheelEncoder[i]);
    MotorPWM[i].setPWM(0);
  }
  delay(1000); // Wait for Encoder to stabilize
  for (int i = 0; i < NumMotors; i++)
  {
    WheelEncoder[i].resetCounter();
  }

  return true;
}

// Init speed controller
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

// Init platformspecific kinematics
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

// Speed controller task
void speedControllerTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(sampleTime);

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
    for (int i = 0; i < 3; i++)
    {
      if (robotSpeed[i] > robotSpeedSetpoint[i])
      {
        robotSpeed[i] = robotSpeed[i] - (robotSpeedAcc[i] * 0.005);
      }
      else if (robotSpeed[i] < robotSpeedSetpoint[i])
      {
        robotSpeed[i] = robotSpeed[i] + (robotSpeedAcc[i] * 0.005);
      }

      if (robotSpeed[i] > robotSpeedMax[i])
      {
        robotSpeed[i] = robotSpeedMax[i];
      }
      else if (robotSpeed[i] < -robotSpeedMax[i])
      {
        robotSpeed[i] = -robotSpeedMax[i];
      }

      if (robotSpeed[i] < 0.000001 && robotSpeed[i] > -0.000001)
      {
        robotSpeed[i] = 0;
      }
    }
    wheelSpeedSetpoint = (1 / wheelRadius) * kinematik * robotSpeed;
    for (int i = 0; i < NumMotors; i++)
    {
      if (abs(wheelSpeedSetpoint[i]) < 0.15)
      {
        wheelSpeedSetpoint[i] = 0;
      }
    }

    for (int i = 0; i < NumMotors; i++)
    {
      speedControllerParam[i].setpoint = wheelSpeedSetpoint(i, 0);
      encoderNew[i] = EncoderDir[i] * WheelEncoder[i].getCount();

      inputsum[i] -= inputs[i][windowIndex];
      inputs[i][windowIndex] = ((((encoderNew[i] - encoderOld[i]) * incrementsToRad) / sampleTime) * 1000);
      inputsum[i] += inputs[i][windowIndex];

      speedControllerParam[i].input = inputsum[i] / windowSize; // Filtered

      speedController[i]->run();

      MotorPWM[i].setPWM(MotorDir[i] * speedControllerParam[i].output * Rad2PWM); // Raw
      encoderOld[i] = encoderNew[i];
    }
    for (int i = 0; i < NumMotors; i++)
    {
      robotWheelSpeed[i] = inputs[i][windowIndex];
    }

    robotOdomSpeed = (wheelRadius * kinematikInv * robotWheelSpeed);

    robotOdomSpeed = robot_vel_to_world_vel(robotOdom[2], robotOdomSpeed); // Conversion Velocity in robotcoordinates to velocity in worldcoordinates

    robotOdom[0] = robotOdomSpeed[0] * sampleTime / 1000 + robotOdom[0];
    robotOdom[1] = robotOdomSpeed[1] * sampleTime / 1000 + robotOdom[1];
    robotOdom[2] = robotOdomSpeed[2] * sampleTime / 1000 + robotOdom[2];
    windowIndex = (windowIndex + 1) % windowSize;
  }
}

// Logger task
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

#ifdef ROS_EN
// Set tf data
void setTfData()
{
  tfData.header.frame_id = "/odom";
  tfData.child_frame_id = "/base_link";
  tfData.transform.translation.x = robotOdom[0];
  tfData.transform.translation.y = robotOdom[1];
  euler_to_quat(0, 0, robotOdom[2], rosQuaternion);
  tfData.transform.rotation.x = (double)rosQuaternion[1];
  tfData.transform.rotation.y = (double)rosQuaternion[2];
  tfData.transform.rotation.z = (double)rosQuaternion[3];
  tfData.transform.rotation.w = (double)rosQuaternion[0];
  tfData.header.stamp = nh.now();

  messageTf.transforms_length = 1;
  messageTf.transforms = &tfData;
}

// Set static tf data
void setTfStaticData()
{
  tfStaticData.header.frame_id = "/base_link";
  tfStaticData.child_frame_id = "/laser";
  tfStaticData.transform.translation.x = 0;
  tfStaticData.transform.translation.y = 0;
  tfStaticData.transform.translation.z = 0.08;
  tfStaticData.transform.rotation.x = 0;
  tfStaticData.transform.rotation.y = 0;
  tfStaticData.transform.rotation.z = 0;
  tfStaticData.transform.rotation.w = 1;
  tfStaticData.header.stamp = nh.now();

  messageTfStatic.transforms_length = 1;
  messageTfStatic.transforms = &tfStaticData;
}
#endif ROS_EN

// Setup
void setup()
{
  Serial.begin(115200);
  initHardware();
  initPID();
  initMatrix();

  xTaskCreate(
      speedControllerTask,   /* Task function. */
      "speedControllerTask", /* String with name of task. */
      100000,                /* Stack size in bytes. */
      NULL,                  /* Parameter passed as input of the task */
      5,                     /* Priority of the task. */
      NULL);                 /* Task handle. */
  xTaskCreate(
      loggerTask,   /* Task function. */
      "loggerTask", /* String with name of task. */
      20000,        /* Stack size in bytes. */
      NULL,         /* Parameter passed as input of the task */
      1,            /* Priority of the task. */
      NULL);        /* Task handle. */

#ifdef ROS_EN
  nh.initNode();
  nh.advertise(tf_pub);
  nh.advertise(tf_static_pub);
  nh.subscribe(sub);
#endif // ROS_EN
}

void loop()
{
  digitalWrite(EnablePIN, HIGH); // Enable motors

  // #############-USER-CODE-START-#####################
  // double tx = 0.0, ty = 0.0, theta = 0.0;
  // robotSpeedSetpoint << tx, ty, theta;

  // #############-USER-CODE-END-#####################

#ifdef ROS_EN
  setTfData();
  setTfStaticData();
  tf_pub.publish(&messageTf);
  tf_static_pub.publish(&messageTfStatic);
  nh.spinOnce();
#endif // ROS_EN

  delay(10);
}