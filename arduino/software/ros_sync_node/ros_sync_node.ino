
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

ros::NodeHandle nh;
std_msgs::Time msg;

// Publishers
ros::Publisher rate_180Hz_pub("sync/rate_180Hz", &msg);
ros::Publisher rate_60Hz_pub("sync/rate_60Hz", &msg);
ros::Publisher rate_30Hz_pub("sync/rate_30Hz", &msg);

// Pin Assignments
int pin_180Hz = 13;
int pin_60Hz = 11;
int pin_30Hz = 9;

// Callback for handling sensor health
uint8_t system_health_ = sync_msgs::sensorHealth::UNKNOWN;
uint32_t _t_last_sensors_msg_ = 0;

// LED State Tracker
class LED_StateTracker {
public:

  enum LED_MODE {
    Solid,
    Flashing
  };

  void set_mode(LED_MODE mode) {
    led_mode_ = mode;
  };
  void set_color(uint8_t red, uint8_t green, uint8_t blue) {
    red_ = red;
    green_ = green;
    blue_ = blue;
  };
  void set_max_brightness(uint8_t max_brightness) {
    max_brightness_ = max_brightness;
  };
  uint32_t get_color() {
    uint8_t max_brightness = max_brightness_;

    if (millis() > t_next_change_) {
      t_next_change_ = millis() + 500;
      led_flash_state_ = !led_flash_state_;
    }

    switch (led_mode_) {
      case (LED_MODE::Flashing):

        // Change brightness around
        if (led_flash_state_) {
          max_brightness = max_brightness_ / 10;
        }
        break;

      case (LED_MODE::Solid):
      default:
        // do nothing
        break;
    }

    // Limit each channel to MAX_BRIGHTNESS
    uint8_t red = min(red_, max_brightness);
    uint8_t green = min(green_, max_brightness);
    uint8_t blue = min(blue_, max_brightness);

    // Combine the limited channels back into the color
    return ((uint32_t)red << 16) | ((uint32_t)green << 8) | blue;
  };

private:

  LED_MODE led_mode_ = LED_MODE::Solid;

  uint8_t red_ = 0;
  uint8_t green_ = 0;
  uint8_t blue_ = 0;

  uint32_t max_brightness_ = 0xFF;
  uint32_t t_next_change_ = 0;
  bool led_flash_state_ = 0;

} ledState_;

void systemHealth_Callback(const sync_msgs::sensorHealthArray& msg) {
  system_health_ = msg.system;
  _t_last_sensors_msg_ = millis();
  return;
}

void loggerStatus_Callback(const std_msgs::Bool& msg) {
  if (msg.data) {
    ledState_.set_mode(LED_StateTracker::LED_MODE::Flashing);
  } else {
    ledState_.set_mode(LED_StateTracker::LED_MODE::Solid);
  }

  return;
}

ros::Subscriber<sync_msgs::sensorHealthArray> health_sub("sensor_status", &systemHealth_Callback);
ros::Subscriber<std_msgs::Bool> recordStart_sub("record_rosbag/status", &loggerStatus_Callback);

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
  nh.subscribe(recordStart_sub);

  // Setup the pins
  pinMode(pin_180Hz, OUTPUT);
  digitalWrite(pin_180Hz, HIGH);
  pinMode(pin_60Hz, OUTPUT);
  digitalWrite(pin_60Hz, HIGH);
  pinMode(pin_30Hz, OUTPUT);
  digitalWrite(pin_30Hz, HIGH);

  // Initialization complete
  nh.loginfo("Sync node hardware initialized");
  core0_init = true;
}

void loop() {

  static const unsigned int us_delay = 2778;  // Time in us of 180 Hz / 2
  static unsigned int counter = 0;

  unsigned long loop_start = micros();

  // Start of frame
  msg.data = nh.now();
  
  digitalWrite(pin_180Hz, LOW);                          // 180 Hz (IMU is on falling edge)
  if ((counter % 3) == 0) digitalWrite(pin_60Hz, HIGH);  //  60 Hz (Thermal is on rising edge)
  if ((counter % 6) == 0) digitalWrite(pin_30Hz, LOW);   //  30 Hz (EO is on falling edge)

  // Publish data (All messages should have the same time)
  rate_180Hz_pub.publish(&msg);
  if ((counter % 3) == 0) rate_60Hz_pub.publish(&msg);
  if ((counter % 6) == 0) rate_30Hz_pub.publish(&msg);

  // Wait a bit
  while (micros() - loop_start < us_delay) {
    // Update ROS
    nh.spinOnce();
  }

  // Invert pins
  digitalWrite(pin_180Hz, HIGH);
  if ((counter % 3) == 1) digitalWrite(pin_60Hz, LOW);
  if ((counter % 6) == 3) digitalWrite(pin_30Hz, HIGH);

  // Loop timing and counting
  counter++;

  while (micros() - loop_start < us_delay + us_delay) {
    // Update ROS
    nh.spinOnce();
  }
}

void setup1() {
  // Wait for core0 to initialize
  do {
    // do nothing
    delay(1000);
  } while (core0_init == 0);

  // Init LEDs
  init_leds();

  // core1 intialized
}

void loop1() {

  // Set the LED colour

  if (nh.connected()) {
    if (millis() - 2UL * 1000 > _t_last_sensors_msg_) {
      // Timeout
      ledState_.set_color(200, 200, 200);  // White
    } else if (system_health_ == sync_msgs::sensorHealth::HEALTHY) {
      // Sensors are all running nominally
      ledState_.set_color(0, 255, 0);  // Green
    } else if (system_health_ == sync_msgs::sensorHealth::UNHEALTHY) {
      // ROS is running, but sensors aren't at the correct rate
      ledState_.set_color(255, 100, 0);  // Amber
    } else {
      // Sensors are missing or state is unknown
      ledState_.set_color(255, 0, 0);  // Red
    }
  } else {
    // No connection to ROS
    ledState_.set_color(0, 0, 255);                         // Blue
    ledState_.set_mode(LED_StateTracker::LED_MODE::Solid);  // Assuming logging has stopped
  }

  // Update LEDs
  update_leds(ledState_.get_color());

  // Sleep for a little bit
  delay(20);
}
