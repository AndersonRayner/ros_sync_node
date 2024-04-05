# ROS Sync Node
Code for running a hardware sync node with an Arduino.

## Hardware
[Waveshare RP2040-Zero](https://www.waveshare.com/rp2040-zero.htm)

[Custom Breakout Board](https://github.com/AndersonRayner/ros_sync_node_hw)

## Features
- Create a sync signal at 60 Hz, 30 Hz and 10 Hz
- Outputs ROS messages `std_msgs/Time.msg` of the time the sync signals were triggered
  - Interfaces via `rosserial` with a 115200 baud rate
    - There might be an issue with the lastest version of the rosserial library in Arduino.  If it fails to compile, revert back to a previous version
- RGB LED control based on the output of `sensor_status` topic
  - See `sync_node/config` for example config file and `sync_node/launch/test.launch` for a test launch file
  - Colors
    - Blue: No ROS connection
    - Red: Sensor fail
    - Amber: Sensor rate error
    - Green: System Nominal 
- RGB LED control based on feedback of robag logging
  - Publish `std_msg::Bool` to `record_rosbag/status`
    - Flashing: Logging active
    - Solid: Logging inactive
  
## Dependencies
### Arduino Side
Libraries (available from the Arduino Library Manager)
- `rosserial`
- `Adafruit NeoPixel` 

### ROS Side
- `rosserial`
