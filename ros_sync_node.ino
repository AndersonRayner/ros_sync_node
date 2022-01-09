
// Code for outputting sync signals.
//    Designed for falling edge
//
//   60 Hz - Pin 8
//   30 Hz - Pin 9

#include "ros.h"
#include "std_msgs/Time.h"

ros::NodeHandle  nh;

std_msgs::Time msg_60Hz;
std_msgs::Time msg_30Hz;

ros::Publisher rate_60Hz_pub("sync/rate_60Hz", &msg_60Hz);
ros::Publisher rate_30Hz_pub("sync/rate_30Hz", &msg_30Hz);

void setup() {
  
  // Setup the pins
  pinMode(8, OUTPUT);   digitalWrite(8, HIGH);
  pinMode(9, OUTPUT);   digitalWrite(9, HIGH);

  // Initialise ROS Node Handler
  nh.initNode();

  // Publishers
  nh.advertise(rate_60Hz_pub);
  nh.advertise(rate_30Hz_pub);
  
}

void loop() {
  
  static const unsigned int us_delay = 8333;  // Time in us of 60 Hz / 2 
  static unsigned int counter = 0;

  unsigned long loop_start = micros();

  // 60 Hz
  // Write pins high
  digitalWrite(8, LOW);

  msg_60Hz.data = nh.now();
  rate_60Hz_pub.publish(&msg_60Hz);

  // Lower Rates
  if ((counter % 2) == 0)
  {
      digitalWrite(9, LOW);

      msg_30Hz.data = nh.now();
      rate_30Hz_pub.publish(&msg_30Hz);

      digitalWrite(13, LOW); // LED Pin

  }

  // Update ROS
  nh.spinOnce();

  // Wait a bit
  delayMicroseconds(us_delay);

  // Write pins low
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(13, HIGH);

  // Loop timing and counting
  counter++;

  while (micros() < loop_start + us_delay + us_delay)
  {
    // do nothing
  }

}
