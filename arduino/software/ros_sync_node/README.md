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
- LED feedback of state
  - Red: No ROS connection
  - Green: ROS connection
  
## Dependencies
### Arduino Side
Libraries (available from the Arduino Library Manager)
- `rosserial`
- `Adafruit NeoPixel` 

### ROS Side
- `rosserial`
