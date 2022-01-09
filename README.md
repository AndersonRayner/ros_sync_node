# ROS Sync Node
Code for running a hardware sync node with an Arduino.

## Hardware
[Waveshare RP2040-Zero](https://www.waveshare.com/rp2040-zero.htm)

## Features
- Create a sync signal at 60 Hz and 30 Hz
- Output ROS message `std_msgs/Time.msg` of the time the sync signals were triggered
  - Use the `rosserial` library (should be able to get it from the arduino libraries manager)
    - There might be an issue with the lastest version, revert back to a previous one if there's an issue
- (TODO) Command the RGB LED (bonus feature)
  - Red: No ROS connection
  - Green: ROS connection
  
