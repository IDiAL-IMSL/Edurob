#define ROS_EN

#define MECANUM
//#define DIFF
//#define OMNI4
//#define OMNI3

#ifdef ROS_EN
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }
#endif  //ROS_EN

#include <WiFi.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "math.h"

#include <dcpwm.h>        // PWM
#include <ESP_Counter.h>  // Encoder
#include <AutoPID.h>      // PID Library https://github.com/r-downing/AutoPID
#include <Eigen.h>        // Linear math
#include <Eigen/QR>       // Calls inverse, determinant, LU decomp., etc.
using namespace Eigen;    // Eigen related statement; simplifies syntax for declaration of matrices

#ifdef ROS_EN
//ROS
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

//ROS Messages
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#endif ROS_EN

//Project specific headers
#include "parameter.h"





//Hardware
static ESP_Counter WheelEncoder[NumMotors];       //Hardware-Encoder-Units
static DCPWM MotorPWM[NumMotors];                 //Hardware-PWM-Units
static AutoPID* speedController[NumMotors];       //PID-Units
static pidParam speedControllerParam[NumMotors];  //PID-Parameter

//Variables
int64_t encoderOld[NumMotors];        //Last encoder values
int64_t encoderNew[NumMotors];        //Current encoder values
const int windowSize = 5;             //Number of values in moving average window
float inputs[NumMotors][windowSize];  //Input values for moving average
float inputsum[NumMotors];            //Sum of values for moving average
int windowIndex = 0;                  //Currently accessed cell of inputs[motornum][x];

//Setpoints
static double setpointSpeed[NumMotors];  //Motorspeed setpoints

//Matrix
MatrixXd kinematik(4, 3);     //Kinematics matrix for differential
MatrixXd kinematikInv(3, 4);  //Inverse Kinematics matrix for differential
Vector3d robotSpeedSetpoint;  //Vector with translationional and rotational robot speeds in m/s and rad/s  (X(m/s), Y(m/s), Z(rad/s))
Vector3d robotSpeed;          //Vector with translationional and rotational robot speeds in m/s and rad/s  (X(m/s), Y(m/s), Z(rad/s))
Vector3d robotSpeedMax;       //Vector with translationional and rotational robot speeds in m/s and rad/s  (X(m/s), Y(m/s), Z(rad/s))
Vector3d robotSpeedAcc;       //Vector with translationional and rotational robot speeds in m/s and rad/s  (X(m/s), Y(m/s), Z(rad/s))
Vector4d wheelSpeedSetpoint;  //Wheel speeds in rad/s
Vector3d robotOdom;           //Odometry
Vector4d robotOdomSpeed;      //Odometry
double rosQuaternion[4];      //Quaternion for ROS transform message

#ifdef ROS_EN
//ROS
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;

geometry_msgs__msg__TransformStamped tfMessage[2];
rcl_subscription_t twistSubscriber;
geometry_msgs__msg__Twist twistMessage;

tf2_msgs__msg__TFMessage tf_message;
rcl_publisher_t publisher;
#endif //ROS_EN


//Error function in case of unhandeld ros-error
void error_loop() {
  while (1) {
    Serial.println("ERR");
    delay(100);
  }
}

//Convert Eulerdegrees to quaternion
const void euler_to_quat(float x, float y, float z, double* q) {
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

//Rotate 2D-Vector
const void rotate2D(float r, double& x, double& y) {
  double temp = x * cos(r) - y * sin(r);
  y = x * sin(r) + y * cos(r);
  x = temp;
}

//Twist message cb
void subscription_callback(const void* msgin) {
  const geometry_msgs__msg__Twist* msga = (const geometry_msgs__msg__Twist*)msgin;
  robotSpeedSetpoint << msga->linear.x, msga->linear.y, msga->angular.z;
}

//Print content of Eigen::MatrixXd
void print_mtxd(const Eigen::MatrixXd& X) {
  int i, j, nrow, ncol;

  nrow = X.rows();
  ncol = X.cols();

  Serial.print("nrow: ");
  Serial.println(nrow);
  Serial.print("ncol: ");
  Serial.println(ncol);
  Serial.println();

  for (i = 0; i < nrow; i++) {
    for (j = 0; j < ncol; j++) {
      Serial.print(X(i, j), 6);  // print 6 decimal places
      Serial.print(", ");
    }
    Serial.println();
  }
  Serial.println();
}

//Init Encoder- and PWM-Units
bool initHardware() {
  pinMode(EnablePIN, OUTPUT);    // Motor enable signal
  digitalWrite(EnablePIN, LOW);  //Disable motors
  for (int i = 0; i < NumMotors; i++) {
    if (!WheelEncoder[i].init(i, Enc_A[i], Enc_B[i])) {
      return false;
    }
  }

  int currentChannel = 0;
  for (int i = 0; i < NumMotors; i++) {
    if (!MotorPWM[i].init(30000, 10, currentChannel, currentChannel + 1, PWM_A[i], PWM_B[i])) {
      return false;
    }
    MotorPWM[i].setPWM(0);  //Set initial speed to 0
    currentChannel += 2;
  }

  return true;
}

//Init speed controller
void initPID() {
  for (int i = 0; i < NumMotors; i++) {
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
    speedController[i]->setBangBang(0, 0);  //Disable BangBang-Controll
  }
}

//Init platformspecific kinematics
void initMatrix() {

#ifdef MECANUM
  //Mecanum
  kinematik << 1, 1, (l1 + l2),
    1, -1, -(l1 + l2),
    1, -1, (l1 + l2),
    1, 1, -(l1 + l2);

  kinematikInv << 1, 1, 1, 1,
    1, -1, 1, -1,
    1.0 / (l1 + l2), -1.0 / (l1 + l2), 1.0 / (l1 + l2), -1.0 / (l1 + l2);

  kinematik = 1 * kinematik;
  kinematikInv = (1 / 4.0) * kinematikInv;
#endif

#ifdef DIFF
  //Diff
  kinematik << 1, 0, l2,
    1, 0, -l2,
    1, 0, l2,
    1, 0, -l2;

  kinematikInv << 1, 1, 1, 1,
    0, 0, 0, 0,
    1.0 / l2, -1.0 / l2, 1.0 / l2, -1.0 / l2;

  kinematik = 1 * kinematik;
  kinematikInv = (1 / 4.0) * kinematikInv;
#endif

#ifdef OMNI4
  //Omni 4 Wheels
  kinematik << (1), -(1), (sqrt(2) * 0.08305),
    (1), (1), -(sqrt(2) * 0.08305),
    (1), (1), (sqrt(2) * 0.08305),
    (1), -(1), -(sqrt(2) * 0.08305);

  kinematikInv << sqrt(2) / 2, sqrt(2) / 2, sqrt(2) / 2, sqrt(2) / 2,
    -sqrt(2) / 2, sqrt(2) / 2, sqrt(2) / 2, -sqrt(2) / 2,
    1 / (2 * (sqrt(2) * 0.08305)), -1 / (2 * (sqrt(2) * 0.08305)), 1 / (2 * (sqrt(2) * 0.08305)), -1 / (2 * (sqrt(2) * 0.08305));

  kinematik = ((sqrt(2) / (2))) * kinematik;
  kinematikInv = (0.5) * kinematikInv;
#endif

  robotSpeedSetpoint << 0.0,
    0.0,
    0.0;

  robotSpeedAcc << maxTAccel, maxTAccel, maxRAccel;
  robotSpeedMax << maxTSpeed, maxTSpeed, maxRSpeed;
  wheelSpeedSetpoint = (1 / wheelRadius) * kinematik * robotSpeedSetpoint;
}

//Speed controller task
void speedControllerTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(sampleTime);

  for (int i = 0; i < NumMotors; i++) {
    inputsum[i] = 0;
  }

  for (int i = 0; i < NumMotors; i++) {
    for (int k = 0; k < windowSize; k++) {
      inputs[i][k] = 0;
    }
  }

  while (true) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    for (int i = 0; i < 3; i++) {
      if (robotSpeed[i] > robotSpeedSetpoint[i]) {
        robotSpeed[i] = robotSpeed[i] - (robotSpeedAcc[i] * 0.005);
      } else if (robotSpeed[i] < robotSpeedSetpoint[i]) {
        robotSpeed[i] = robotSpeed[i] + (robotSpeedAcc[i] * 0.005);
      }

      if (robotSpeed[i] > robotSpeedMax[i]) {
        robotSpeed[i] = robotSpeedMax[i];
      } else if (robotSpeed[i] < -robotSpeedMax[i]) {
        robotSpeed[i] = -robotSpeedMax[i];
      }
    }
    wheelSpeedSetpoint = (1 / wheelRadius) * kinematik * robotSpeed;
    for (int i = 0; i < NumMotors; i++) {
      if (abs(wheelSpeedSetpoint[i]) < 0.15) {
        wheelSpeedSetpoint[i] = 0;
      }
    }

    for (int i = 0; i < NumMotors; i++) {
      speedControllerParam[i].setpoint = wheelSpeedSetpoint(i, 0);
      encoderNew[i] = EncoderDir[i] * WheelEncoder[i].getCount();

      inputsum[i] -= inputs[i][windowIndex];
      inputs[i][windowIndex] = ((((encoderNew[i] - encoderOld[i]) * incrementsToRad) / sampleTime) * 1000);
      inputsum[i] += inputs[i][windowIndex];

      speedControllerParam[i].input = inputsum[i] / windowSize;  //Filtered

      speedController[i]->run();

      MotorPWM[i].setPWM(MotorDir[i] * speedControllerParam[i].output * Rad2PWM);  //Raw
      encoderOld[i] = encoderNew[i];
    }
    for (int i = 0; i < NumMotors; i++) {
      robotOdomSpeed[i] = inputs[i][windowIndex];
    }
    Eigen::Vector3d helper = (wheelRadius * kinematikInv * robotOdomSpeed);
    rotate2D(robotOdom[2], helper[0], helper[1]);
    robotOdom[0] = helper[0] / 200 + robotOdom[0];
    robotOdom[1] = helper[1] / 200 + robotOdom[1];
    robotOdom[2] = helper[2] / 200 + robotOdom[2];
    windowIndex = (windowIndex + 1) % windowSize;
  }
}

//Logger task
void loggerTask(void* pvParameters) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(5));
    Serial.print("Odometry: Xt= ");
    Serial.print(robotOdom[0]);  //current x-position in meters
    Serial.print(", Yt= ");
    Serial.print(robotOdom[1]);  //current y-position in meters
    Serial.print(", Zr= ");
    Serial.println(robotOdom[2]);  //current z-rotation in radians
  }
}

//Setup
void setup() {
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
  set_microros_wifi_transports("ImslRaspAP", "imslraspapkey", "10.3.141.3", 8888);
  delay(2000);
  allocator = rcl_get_default_allocator();
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create node
  RCCHECK(rclc_node_init_default(&node, "Edurob", "", &support));
  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &twistSubscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &twistSubscriber, &twistMessage, &subscription_callback, ON_NEW_DATA));

  tfMessage[0].header.frame_id =
    micro_ros_string_utilities_set(tfMessage[0].header.frame_id, "/odom");
  tfMessage[0].child_frame_id =
    micro_ros_string_utilities_set(tfMessage[0].child_frame_id, "/base_link");

  tf_message.transforms.size = 1;
  tf_message.transforms.data = tfMessage;
#endif  //ROS_EN
}



void loop() {
  digitalWrite(EnablePIN, HIGH);  //Enable motors

  //#############-USER-CODE-START-#####################
  double tx = 0.0, ty = 0.0, theta = 0.0;
  robotSpeedSetpoint << tx, ty, theta;

  //#############-USER-CODE-END-#####################

#ifdef ROS_EN
  tf_message.transforms.data[0].transform.translation.x = robotOdom[0];
  tf_message.transforms.data[0].transform.translation.y = robotOdom[1];
  euler_to_quat(0, 0, robotOdom[2], rosQuaternion);
  tf_message.transforms.data[0].transform.rotation.x = (double)rosQuaternion[1];
  tf_message.transforms.data[0].transform.rotation.y = (double)rosQuaternion[2];
  tf_message.transforms.data[0].transform.rotation.z = (double)rosQuaternion[3];
  tf_message.transforms.data[0].transform.rotation.w = (double)rosQuaternion[0];
  tf_message.transforms.data[0].header.stamp.nanosec = xTaskGetTickCount() * 1000000 / configTICK_RATE_HZ;
  tf_message.transforms.data[0].header.stamp.sec = xTaskGetTickCount() / configTICK_RATE_HZ;
  RCSOFTCHECK(rcl_publish(&publisher, &tf_message, NULL));
  RCCHECK(rclc_executor_spin_some(&executor, 0));
#endif  //ROS_EN
  delay(100);
}