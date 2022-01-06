# ROS Sync Node
Code for running a hardware sync node with an Arduino.

## Hardware
(Waveshare RP2040-Zero)[https://www.waveshare.com/rp2040-zero.htm]

## Features
- Create a sync signal at 60 Hz and 30 Hz
  - Example (bad) code for this (here)[https://github.com/aerorobotics/ONR_drone_spinnaker_ws/tree/master/Code/Arduino/sync_60Hz] which has a 60 Hz and 10 Hz loop
- Output ROS message `std_msgs/Time.msg` of the time the sync signals were triggered
  - Can use a different message type if you find something better
- Command the RGB LED (bonus feature)
  - Red: No ROS connection
  - Green: ROS connection
  