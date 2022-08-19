
// Code for outputting falling edge sync signals
//    Designed for Waveshare RPi RP2040-Zero
//    Compile as Raspberry Pi Pico
//
//   180 Hz - Pin 13
//    60 Hz - Pin 11
//    30 Hz - Pin  9
// NeoPixel - Pin 16 (built-in)

#include "ros.h"

#include "std_msgs/Time.h"
#include "std_msgs/Bool.h"

#include <Adafruit_NeoPixel.h>

ros::NodeHandle  nh;

std_msgs::Time msg;

// Publishers
ros::Publisher rate_180Hz_pub("sync/rate_180Hz", &msg);
ros::Publisher rate_60Hz_pub("sync/rate_60Hz", &msg);
ros::Publisher rate_30Hz_pub("sync/rate_30Hz", &msg);

// Pin Assignments
int pin_180Hz = 13;
int pin_60Hz  = 11;
int pin_30Hz  =  9;

int led_pin_   = 16;
int led_count_ =  1;

// Callback for handling sensor health
bool sensors_ok = false;
uint32_t _t_last_sensors_ok = 0;

void sensorOk_Callback(const std_msgs::Bool& msg)
{
    sensors_ok = msg.data; 
    _t_last_sensors_ok = millis();
    
    return;
}

ros::Subscriber<std_msgs::Bool> sub("sensors_ok", &sensorOk_Callback);

// LED
Adafruit_NeoPixel strip(led_count_, led_pin_, NEO_GRB + NEO_KHZ800);

// Main 
void setup() {
  
  // Start the serial port
  Serial.begin(115200);

  // Initialise ROS Node Handler
  nh.initNode();

  // Publishers
  nh.advertise(rate_180Hz_pub);
  nh.advertise(rate_60Hz_pub);
  nh.advertise(rate_30Hz_pub);

  //Subscriber
  nh.subscribe(sub);

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
    if ((sensors_ok) &&
        (millis() - 2UL*1000 < _t_last_sensors_ok) )
    {
        // Sensors are all running nominally
        strip.setPixelColor(0, strip.Color(0,   255,   0)); // Green
    }
    else
    {
        // ROS is running, but sensors aren't at the correct rate
        strip.setPixelColor(0, strip.Color(255,  100,   0)); // Amber
    }  
  }
  else
  {
      // No connection to ROS
      strip.setPixelColor(0, strip.Color(255,   0,   0)); // Red
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
