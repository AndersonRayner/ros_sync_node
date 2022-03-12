
// Code for outputting falling edge sync signals
//    Designed for Waveshare RPi RP2040-Zero
//    Compile as Raspberry Pi Pico
//
//    60 Hz - Pin 13
//    30 Hz - Pin 11
//    10 Hz - Pin  9
// NeoPixel - Pin 16 (built-in)

#include "ros.h"
#include "std_msgs/Time.h"

#include <Adafruit_NeoPixel.h>

ros::NodeHandle  nh;

std_msgs::Time msg;

ros::Publisher rate_60Hz_pub("sync/rate_60Hz", &msg);
ros::Publisher rate_30Hz_pub("sync/rate_30Hz", &msg);
ros::Publisher rate_10Hz_pub("sync/rate_10Hz", &msg);

int pin_60Hz = 13;
int pin_30Hz = 11;
int pin_10Hz =  9;

int led_pin_   = 16;
int led_count_ =  1;

Adafruit_NeoPixel strip(led_count_, led_pin_, NEO_GRB + NEO_KHZ800);

void setup() {
  
  // Start the serial port
  Serial.begin(115200);

  // Initialise ROS Node Handler
  nh.initNode();

  // Publishers
  nh.advertise(rate_60Hz_pub);
  nh.advertise(rate_30Hz_pub);
  nh.advertise(rate_10Hz_pub);

  // Start the LED
  strip.begin();
  strip.setBrightness(255);
  strip.setPixelColor(0, strip.Color(0,   0,   255));
  strip.show();
  
  // Setup the pins
  pinMode(pin_60Hz, OUTPUT);   digitalWrite(pin_60Hz, HIGH);
  pinMode(pin_30Hz, OUTPUT);   digitalWrite(pin_30Hz, HIGH);
  pinMode(pin_10Hz, OUTPUT);   digitalWrite(pin_10Hz, HIGH);


  
}

void loop() {
  
  static const unsigned int us_delay = 8333;  // Time in us of 60 Hz / 2 
  static unsigned int counter = 0;

  unsigned long loop_start = micros();

  // Write pins LOW (start of the frame)
  msg.data = nh.now();

  // 60 Hz  
  digitalWrite(pin_60Hz, LOW); 

  // 30 Hz
  if ((counter % 2) == 0) digitalWrite(pin_30Hz, LOW);
  
  // 10 Hz
  if ((counter % 6) == 0) digitalWrite(pin_10Hz, LOW);
  
  // Publish data (All messages should have the same time)
  rate_60Hz_pub.publish(&msg);
  if ((counter % 2) == 0) rate_30Hz_pub.publish(&msg);
  if ((counter % 6) == 0) rate_10Hz_pub.publish(&msg);

  // Wait a bit
  while (micros()-loop_start < us_delay)
  {
     // Update ROS
     nh.spinOnce();
  }
  //delayMicroseconds(us_delay);

  // Write pins high
  digitalWrite(pin_60Hz, HIGH);
  digitalWrite(pin_30Hz, HIGH);
  digitalWrite(pin_10Hz, HIGH);

  // Set the LED colour
  if (nh.connected())
  {
      strip.setPixelColor(0, strip.Color(0,   255,   0));
  } else {
      strip.setPixelColor(0, strip.Color(255,   0,   0));
  }
  strip.show();

  // Loop timing and counting
  counter++;
  
  while (micros()-loop_start < us_delay + us_delay)
  {
    // do nothing

    // Update ROS
     nh.spinOnce();
  }

}
