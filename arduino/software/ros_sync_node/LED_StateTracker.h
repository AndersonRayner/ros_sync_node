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
};
