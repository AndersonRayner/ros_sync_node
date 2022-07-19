
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

ros::Publisher rate_180Hz_pub("sync/rate_180Hz", &msg);
ros::Publisher rate_60Hz_pub("sync/rate_60Hz", &msg);
ros::Publisher rate_30Hz_pub("sync/rate_30Hz", &msg);

int pin_180Hz = 13;
int pin_60Hz  = 11;
int pin_30Hz  =  9;

int led_pin_   = 16;
int led_count_ =  1;

Adafruit_NeoPixel strip(led_count_, led_pin_, NEO_GRB + NEO_KHZ800);

void setup() {
  
  // Start the serial port
  Serial.begin(115200);

  // Initialise ROS Node Handler
  nh.initNode();

  // Publishers
  nh.advertise(rate_180Hz_pub);
  nh.advertise(rate_60Hz_pub);
  nh.advertise(rate_30Hz_pub);

  // Start the LED
  strip.begin();
  strip.setBrightness(255);
  strip.setPixelColor(0, strip.Color(0,   0,   255));
  strip.show();
  
  // Setup the pins
  pinMode(pin_180Hz, OUTPUT);  digitalWrite(pin_180Hz, HIGH);
  pinMode(pin_60Hz,  OUTPUT);  digitalWrite(pin_60Hz,  HIGH);
  pinMode(pin_30Hz,  OUTPUT);  digitalWrite(pin_30Hz,  HIGH);
  
}

void loop() {
  
  static const unsigned int us_delay = 2778;  // Time in us of 180 Hz / 2 
  static unsigned int counter = 0;

  unsigned long loop_start = micros();

  // Write pins LOW (start of the frame)
  msg.data = nh.now();

  // 180 Hz
  digitalWrite(pin_180Hz, LOW);
  
  // 60 Hz  
  if ((counter % 3) == 0) digitalWrite(pin_60Hz, LOW); 

  // 30 Hz
  if ((counter % 6) == 0) digitalWrite(pin_30Hz, LOW);
   
  // Publish data (All messages should have the same time)
  rate_180Hz_pub.publish(&msg);
  if ((counter % 3) == 0) rate_60Hz_pub.publish(&msg);
  if ((counter % 6) == 0) rate_30Hz_pub.publish(&msg);

  // Wait a bit
  while (micros()-loop_start < us_delay)
  {
     // Update ROS
     nh.spinOnce();
  }
  //delayMicroseconds(us_delay);

  // Write pins high
  digitalWrite(pin_180Hz, HIGH);
  digitalWrite(pin_60Hz,  HIGH);
  digitalWrite(pin_30Hz,  HIGH);

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
