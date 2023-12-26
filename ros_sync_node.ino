
// Code for outputting falling edge sync signals
//    Designed for Waveshare RPi RP2040-Zero
//    Compile as Raspberry Pi Pico
//
//   180 Hz - Pin 13
//    60 Hz - Pin 11
//    30 Hz - Pin  9
// NeoPixel - Pin 16 (built-in)

#include <ros.h>

#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>

#include "sync_msgs/sensorHealth.h"
#include "sync_msgs/sensorHealthArray.h"

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

// Callback for handling sensor health
bool system_health_ = false;
uint32_t _t_last_sensors_msg_ = 0;

void systemHealth_Callback(const sync_msgs::sensorHealthArray& msg)
{
    system_health_ = msg.system; 
    _t_last_sensors_msg_ = millis();
    
    return;
}

ros::Subscriber<sync_msgs::sensorHealthArray> health_sub("/sensor_status", &systemHealth_Callback);


// Other
bool core0_init = false;

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

  // Subscribers
  nh.subscribe(health_sub);
 
  // Setup the pins
  pinMode(pin_180Hz, OUTPUT);  digitalWrite(pin_180Hz, HIGH);
  pinMode(pin_60Hz,  OUTPUT);  digitalWrite(pin_60Hz,  HIGH);
  pinMode(pin_30Hz,  OUTPUT);  digitalWrite(pin_30Hz,  HIGH);

  // Init LEDs
  init_leds();

  // Initialization complete
  nh.loginfo("Sync node hardware initialized");
  core0_init = true;

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

  // Write pins high
  digitalWrite(pin_180Hz, HIGH);
  digitalWrite(pin_60Hz,  HIGH);
  digitalWrite(pin_30Hz,  HIGH);

  // Loop timing and counting
  counter++;
  
  while (micros()-loop_start < us_delay + us_delay)
  {
    // Update ROS
     nh.spinOnce();
  }

}

void setup1()
{
  // Wait for the first core to update everything
  while (core0_init == 0)
  {
    // do nothing
    delay(500);
  }

  // core0 intialized

}

void loop1()
{

 // Set the LED colour
  uint32_t color = Adafruit_NeoPixel::Color(0,0,0);

  if (nh.connected())
  {
    if (millis() - 2UL*1000 > _t_last_sensors_msg_)
    {
      // Timeout
      color = Adafruit_NeoPixel::Color(200, 200, 200); // White
    }
    else if (system_health_ == sync_msgs::sensorHealth::HEALTHY)
    {
      // Sensors are all running nominally
      color = Adafruit_NeoPixel::Color(0,255, 0); // Green
    }
    else if (system_health_ == sync_msgs::sensorHealth::UNHEALTHY)
    {
      // ROS is running, but sensors aren't at the correct rate
      color = Adafruit_NeoPixel::Color(255, 100, 0); // Amber
    }
    else
    {
      // Sensors are missing or state is unknown
      color = Adafruit_NeoPixel::Color(255, 0, 0); // Red      
    } 
  }
  else
  {
    // No connection to ROS
    color = Adafruit_NeoPixel::Color(0, 0, 255); // Blue
  }

  // Update LEDs
  update_leds(color);

  // Sleep for a little bit
  delay(200);
  
}
