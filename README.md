# ROS Sync Node
Code for running a hardware sync node with an Arduino.

## Hardware
[Waveshare RP2040-Zero](https://www.waveshare.com/rp2040-zero.htm)

## Features
- Create a sync signal at 60 Hz and 30 Hz
- Outputs ROS messages `std_msgs/Time.msg` of the time the sync signals were triggered
  - Uses the `rosserial` library (available from the arduino libraries manager), with a 115200 baud rate
    - There might be an issue with the lastest version, revert back to a previous one if there's an issue
- LED feedback of state
  - Red: No ROS connection
  - Green: ROS connection
  
