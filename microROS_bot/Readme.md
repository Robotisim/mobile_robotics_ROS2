### Installations
- VScode and Platformfio extension
- MicroROS platformio Library
    ```
    https://github.com/micro-ROS/micro_ros_platformio
    ```
- MicroROS Agent into a workspace and build
    ```
    https://github.com/micro-ROS/micro_ros_setup
    ```
### Setup
- Agent package
    ```
    cd ~/~/microros_ws && colcon build
    source install/setup.bash
    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
    ```

### Running
- Upload MicroROS based code into MicroController

- #### Agent Serial
    - Start Agent
        ```
        ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
        ```
- #### Agent Wifi
    - Start Agent
        ```
        ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
        ```

