## QUICKSTART TO EDUROB V2

### Installation 

Software required:
- Arduino IDE (https://www.arduino.cc/en/software/) - tested with Arduino IDE 2.3.6
- optional: ROS2 Jazzy (https://docs.ros.org/en/jazzy/) - tested with Ubuntu 24.04 

Integration in Arduino IDE:
1. Clone the Repository and move the included library folders in `Edurob/libraries` to the local library folder of Arduino IDE, for instance `~/Arduino/libraries`
4. Insert "Addition boards manager URL": File → Preferences: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
5. Select board: Tools → Board →esp32 →`ESP32S3 Dev Module`
6. Select port: Tools → Port → for instance `/dev/ttyACM0`
7. Open the folder `Edurob` and open the Arduino sketch `Edurob.ino` in Arduino IDE
8. Modify the Arduino sketch according to your own preferences (see Usage below)
9. Verify and upload the Arduino sketch via USB-C port on Edurob V2 and hit the reset button on the robot to start the uploaded code

### Usage

Set configuration of the Edurob V2 in lines 6 to 10 of `parameter.h` to to define the kinematics of the robot and only comment out the one selected configuration (ideally matching the wheel configuration of the robot hardware):
```
// Set EduRob Configuration
#define MECANUM
// #define DIFF
// #define OMNI4
// #define OMNI3
```

Set line 4 of  `parameter.h` to disable or enable a wireless connection to ROS2 device (via micro_ros)

- disable: `//#define ROS_EN` 
	
	Add user code in the corresponding segment inside the loop function (lines 665 to 669) of `Edurob.ino`:
	```
	//#############-USER-CODE-START-#####################
  	// double tx = 0.0, ty = 0.0, theta = 0.0;
  	// robotSpeedSetpoint << tx, ty, theta;
	// #############-USER-CODE-END-#####################
	```

- enable: `#define ROS_EN`
	
	Set ROS2 enabled parameters in lines 14 to 19 of `parameter.h`:

	```
	#ifdef ROS_EN
	#define ROS_DOMAIN_ID 1
	#define WIFISSID "ESPNET"
	#define WIFIPASS "espnetwifikey"
	#define AGENT_IP "10.0.101.253"
	#define AGENT_PORT 8888
	```

	- set ROS_DOMAIN_ID (refers to https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html)
	- set name and password of wifi
	- set ip address and port of micro_ros_agent on ROS2 Jazzy device in same network

	Installation of micro_ros_agent (refers to https://github.com/micro-ROS/micro_ros_setup) on ROS2 Jazzy device in same network in a ROS2 workspace folder (for instance ~/ros2_ws) in a terminal:

	```
	cd ~/ros2_ws
	sudo apt update
	sudo apt install python3-rosdep
	git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
	rosdep update && rosdep install --from-paths src --ignore-src -y
	sudo rosdep init
	rosdep update
	colcon build --symlink-install
	source install/local_setup.bash
	ros2 run micro_ros_setup create_agent_ws.sh
	ros2 run micro_ros_setup build_agent.sh
	source install/local_setup.bash
	```

	Start of micro_ros_agent on ROS2 Jazzy device with udp4 and defined AGENT_PORT parameters in a terminal:

	```
	ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
	```

	The terminal should output:

	```
	[1761729504.052943] info     | UDPv4AgentLinux.cpp | init                     | running...             | port: 8888
	[1761729504.055070] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
	```

	Press the reset button on the robot. The terminal output will change if successful:

	```
	[1761729504.052943] info     | UDPv4AgentLinux.cpp | init                     | running...             | port: 8888
	[1761729504.055070] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
	[1761729843.354045] info     | Root.cpp           | create_client            | create                 | client_key: 0x7A4912AF, session_id: 0x81
	[1761729843.355923] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x7A4912AF, address: 10.0.109.1:47138
	[1761729843.510983] info     | ProxyClient.cpp    | create_participant       | participant created    | client_key: 0x7A4912AF, participant_id: 0x000(1)
	[1761729843.574304] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x7A4912AF, topic_id: 0x000(2), participant_id: 0x000(1)
	[1761729843.586440] info     | ProxyClient.cpp    | create_subscriber        | subscriber created     | client_key: 0x7A4912AF, subscriber_id: 0x000(4), participant_id: 0x000(1)
	[1761729843.594849] info     | ProxyClient.cpp    | create_datareader        | datareader created     | client_key: 0x7A4912AF, datareader_id: 0x000(6), subscriber_id: 0x000(4)
	[1761729843.601925] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x7A4912AF, topic_id: 0x001(2), participant_id: 0x000(1)
	[1761729843.607856] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x7A4912AF, publisher_id: 0x000(3), participant_id: 0x000(1)
	[1761729843.614818] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x7A4912AF, datawriter_id: 0x000(5), publisher_id: 0x000(3)
	```

	The ROS2 terminal command `ros2 topic list` will show:

	```
	/cmd_vel
	/parameter_events
	/rosout
	/tf
	```

	The ROS2 terminal command `ros2 node list` will output `/EduRob_Node`. A following command `ros2 node info /EduRob_Node` will show that the node `/EduRob_Node` subscribed to the topic `/cmd_vel` and publishes the topic `/tf` as seen in the topic list before:

	```
	/EduRob_Node
	  Subscribers:
		/cmd_vel: geometry_msgs/msg/Twist
	  Publishers:
		/tf: tf2_msgs/msg/TFMessage
	```

	Since transformations are published via `/tf` and driving commands are sent in ROS2 via `/cmd_vel`, the robot is ready to be controlled, among other things, via a teleoperation node such as teleop_twist_keyboard.
