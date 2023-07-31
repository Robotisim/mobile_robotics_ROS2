# Installation of Micro Ros

Installations :
```
sudo apt install python3-rosdep
sudo apt-get install python3-pip
```
 ### Step#1 ROS 2 Installation (Humble)
```
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#install-ros-2-packages
```

### Step#2 ESP IDF Installation 
- Esp IDF extention using VSCode:
```
https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md
```
Try different examples like (Hello World,Talker listner). It can be done on VScode by adding the examples by {View-Command Pallet -"select the Esp Idf Examples"} then create the project using the examples in specific workspace OR using Terminal.

### Step#3 MICRO ROS INSTALLATION

Source the ROS 2 installation
```
source /opt/ros/$ROS_DISTRO/setup.bash
```
Notes :
-  source: This is the command used to execute commands.It reads and runs the commands in the specified file as if they were typed directly into the terminal.

 - /opt/ros/: This is the default installation directory for ROS .With this command, we are specifying the path to the ROS installation directory.

- $ROS_DISTRO: This is a variable that holds the name of the ROS distribution (version) you want to use. For example, if your desired ROS version is "humble," the $ROS_DISTRO variable will be replaced with "humble" in the command.

- setup.bash: This is a script file that contains the environment setup commands for the specified ROS distribution. 

### Step # 4 Create a workspace and download the micro-ROS tools
This will create a micro-ros workspace which will include Micro Ros tools
```
mkdir mros_ws
```
Notes :

 - mkdir microros_ws: The "mkdir" command is short for "make directory." It is used to create a new directory (folder) with the specified name.
```
cd mros_ws
```
Notes:

- cd mros_ws: The "cd" command is short for "change directory." It is used to change the current working directory to the specified directory.

- $ROS_DISTRO = It will pickup the version of ROS we want to use for example in this case it will replace $ROS_DISTRO with "Humble" as its already been downloaded in the system.
```
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```

### Step#5 Update dependencies using rosdep
These commands are used to update package lists and dependencies for software installed on the system.

Notes :
-  sudo apt update : Running this command ensures that you have the most up-to-date information about packages available for installation.
-  rosdep update: It fetches the latest rules for resolving package dependencies and ensures that you have the most current information available when building ROS packages.
```
sudo apt update && rosdep update
rosdep install --from-path src --ignore-src -y
```
This command ensures that all the necassary dependiencies are installed in ROS with in the workspace.It runs inside catkin or colcon workspace to resolve and install dependencies for the packages located in the "src" directory.

Notes : 
- rosdep install: To install system dependencies for ROS packages.
- --from-path src: It specifies that the packages for which dependencies should be installed are located in the "src" directory of the current workspace.
- --ignore-src: This argument tells rosdep to ignore intalling any ROS package dependencies from the source (e.g., via Git or Mercurial repositories) and only consider system dependencies for installation.
- -y: This argument says "yes" to the installation process and allows to run the command without requiring user input.


### Step#6 Building packages and setting up your workspace to use the built packages:
By running these two commands, you are building the ROS 2 packages in your workspace using colcon and setting up your terminal session to use the built packages with the source command. This is a standard workflow for ROS 2 development.

Notes :
- colcon build:colcon is a build tool used in ROS 2 for building packages and workspaces.
```
colcon build
source install/local_setup.bash
```
After running colcon build, the built artifacts will be placed in the "install" directory inside your workspace. The local_setup.bash script in the "install" directory sets up your shell environment to use the built packages from the workspace.

### Step#7 Creating a new firmware workspace for ESP32
This command is used to create a new firmware workspace for micro-ROS on an ESP32 microcontroller running FreeRTOS as the Real-Time Operating System (RTOS).After executing this command a folder called "firmware" will be created in the workspace along with the required code for building micro-ros app.
```
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
```
My results:

[notice] A new release of pip is available: 23.2 -> 23.2.1

[notice] To update, run: pip install --upgrade pip

 ## Using Example: INT32 publisher 

### Step#8 Configuring Micro-ros firmware:
The configuration step will set up the main micro-ROS firmware, after executing the command it will help us to configure the files for our project.Then enter the project name which will include the code and Device IP address ,it could be either(computer, laptop etc)

Notes : 
- The example used here is Int32 Publisher so the PROJECT NAME : int32 Publisher and the specific Ip address.

```
ros2 run micro_ros_setup configure_firmware.sh [PROJECT NAME] -t udp -i [LOCAL MACHINE IP ADDRESS] -p 8888
```
IP address of the devise can be extracted by:
```
ifcongig
```
Notes :
- I used the following entries in my command:
```
ros2 run micro_ros_setup configure_firmware.sh int32_publisher -t udp -i 192.168.1.88 -p 8888
```

### Configure Wi-Fi Credentials:
Set up the Wi-Fi credentials (SSID and password) to connect to your local Wi-Fi network. 
```
ros2 run micro_ros_setup build_firmware.sh menuconfig
```
Notes :
- ros2 run micro_ros_setup: This is the ros2 run command to execute a specific package in the ROS 2 ecosystem
- menuconfig: This argument is passed to the script and specifies the configuration interface.

- Following command gives access to "EspressifIOT Development framework configration" window, configure WiFi settings there by accessing Micro ros transport settings - wifi settings - then change wifi SSID to the one your computer has ,I changed it to "Gigaclear_Zafar" and then insert the password. hit the "S" key to save and then enter and press "Q" to quit the window.

### Building micro Ros firmware:
By running build_firmware.sh the firmware is built
```
ros2 run micro_ros_setup build_firmware.sh

pip install esptool
```
### Flashing Micro Ros firmware:
To flash ESP32 first connect it with the computer through usb and then run this command, and it will start to flash.

ros2 run micro_ros_setup flash_firmware.sh

### Step#9: Building Micro Ros Agent:
Notes: 

 why Building micro-ROS-Agent is necessary?
1. Building the micro-ROS-Agent is necessary because it serves as a crucial bridge between your embedded system (microcontroller or resource-constrained device) and the ROS 2 ecosystem. The micro-ROS-Agent enables communication and interaction between these two environments, allowing your embedded device to become a part of the ROS 2 network and leverage the capabilities of ROS 2.
-  2.micro-ROS consists of a node running on the microcontroller and an agent running on the host computer. The agent handles the interface between the node and the rest of the ROS2 stack. This allows the ROS2 stack and microcontroller node to publish and subscribe to each other as if the node was like any other ROS2 node.

Following commands will allow to create Ros agent so that the device can start communicating(publish & subscribe) with ros 2 environment.
```
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
ros2 run micro_ros_agent micro_ros_agent [parameters]
ros2 run micro_ros_setup create_agent_ws.sh
```
### Building the agent:
```
ros2 run micro_ros_setup build_agent.sh
```
### Source the installiation:
```
source install/local_setup.sh
```
### Run micro ros agent:
Once ESP32 connected to the Wi-Fi network and the micro-ROS-Agent running with the correct p arameters, the ESP32 should be able to communicate wirelessly with the ROS 2 ecosystem via micro-ROS.
```
ros2 run micro_ros_agent micro_ros_agent [parameters]
```
I used the command:
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
The command runs the micro-ROS-Agent with UDP (User Datagram Protocol) transport on port 8888. 

Notes:
- ros2 run: This is the command used to run a ROS 2 node. It allows you to execute a specified ROS 2 package and node.

- micro_ros_agent: This is the name of the ROS 2 package that contains the micro-ROS-Agent node.

- udp4: This is an argument specifying that the transport protocol to be used is UDPv4. UDP is a lightweight, connectionless protocol often used for real-time communication in embedded systems.

- --port 8888: This argument specifies the port number on which the micro-ROS-Agent will listen for incoming UDP traffic. In this case, it's set to port 8888.

IMPORTANT: When you run this command, the micro-ROS-Agent will start and listen for UDP packets on port 8888. It will act as the intermediary between your embedded system (which should be running a micro-ROS-enabled firmware) and the ROS 2 network. Messages sent from your embedded system will be received by the micro-ROS-Agent and then forwarded to the appropriate ROS 2 nodes. Similarly, messages from ROS 2 nodes will be relayed to your embedded system through the micro-ROS-Agent.

### Results:
- tried running the publisher wirelessly on esp32 (After pressing the restart button on esp32 it did not work).

- Tried to run the publisher with USB, with the help of following command and it results in success.

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```
press the enable button and it will start giving the results:

### Step#10 Testing:
By typing Ros2 topic list

### Results:
mehtab@mehtab-K55VD:~/microros_ws$ ros2 topic list

*/freertos_int32_publisher*

*/parameter_events*

*/rosout*

*mehtab@mehtab-K55VD:~/microros_ws$ /freertos_int32_publisher*

*/parameter_events*

*/rosout*

**The above results shows that int32 publishers is publishing.**

### Subscribing to micro-ROS topic
```
ros2 topic echo /freertos_int32_publisher
```
results shows that data is being subscribed by the microcontroller.





