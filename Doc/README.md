# Getting Started
[Homepage](https://www.imsl.fh-dortmund.de/mobile-roboter/edurob/)
## Initial Setup

- Install Arduino IDE 2.1.0 (or later): https://www.arduino.cc/en/software
- Install the Arduino Core for ESP32: https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
- Copy the contents of the "Edurob\libraries" folder into your local libraries folder (e.g. "C:\Users\NAME\Documents\Arduino\libraries")
- Open the project ("Edurob.ino") inside the Arduino IDE
- Select "ESP32 Dev Module" as Board under "Tools/Board/esp32/ESP32 Dev Module"
- Depending on your hardware configuration (Mecanum, Differential drive, 3-Wheel Omni or 4-Wheel Omni) you have to select the corresponding software configuration by uncommenting **only one** of the following statements

    `#define MECANUM`
    
    `#define DIFF`

    `#define OMNI4`

    `#define OMNI3`
- (Optional for ROS) Install ROS2 "Humble Hawksbill": https://docs.ros.org/en/humble/Installation.html
- (Optional for ROS) Build the micro-ROS package: https://github.com/micro-ROS/micro_ros_setup#building
- (Optional for ROS) Build the micro-ROS agent: https://github.com/micro-ROS/micro_ros_setup#building-micro-ros-agent
- (Optional for ROS) Uncomment `#define ROS_EN` to enable support for ROS in the software and add your wifi credentials inside the setup-function `set_microros_wifi_transports("APNAME", "WIFIKEY", "MICRO_ROS_IP", 8888);`
- Insert the powerbank into the Edubot
- Connect your Edurob with a micro USB-cable to your computer
- Select the corresponding upload port under "Tools/Port/COMx"
- Set the upload speed to 921600 under "Tools/Upload Speed/921600"
- Your configuration should look like this:

    ![](/Doc/Pictures/ArduinoConfig.png "")
- Select "Upload"
- (Optional for ROS) Start your micro-Ros agent: `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`
- (Optional for ROS) To controll your Edubot use `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

---

## Next Steps
- Add your user code in the corresponding segment inside the loop-function
  
  `//#############-USER-CODE-START-#####################`
  
  `double tx = 0.0, ty = 0.0, theta = 0.0;`
  
  `robotSpeedSetpoint << tx, ty, theta;`

  `//#############-USER-CODE-END-#####################`
- Set the robot speed by changing the values inside the "robotSpeedSetpoint"-Vector