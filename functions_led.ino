
// neopixel setup
int led_pin_internal_     = 16;
int led_pin_external_     = 15;
uint16_t n_leds_internal_ = 1;
uint16_t n_leds_external_ = 1;

// neopixel objects
Adafruit_NeoPixel strip_internal_;
Adafruit_NeoPixel strip_external_;

void init_leds()
{

  // Start the LED strips
  strip_internal_.setPin(led_pin_internal_);
  strip_internal_.updateLength(n_leds_internal_);
  strip_internal_.updateType(NEO_GRB + NEO_KHZ800);

  strip_external_.setPin(led_pin_internal_);
  strip_external_.updateLength(n_leds_internal_);
  strip_external_.updateType(NEO_GRB + NEO_KHZ800);

  // Start strip
  strip_internal_.begin();
  strip_internal_.setBrightness(255);

  strip_external_.begin();
  strip_external_.setBrightness(255);


  // Flash LEDs
  update_leds(Adafruit_NeoPixel::Color(0xFF,0x00,0x00)); delay(500);
  update_leds(Adafruit_NeoPixel::Color(0x00,0xFF,0x00)); delay(500);
  update_leds(Adafruit_NeoPixel::Color(0x00,0x00,0xFF)); delay(500);
  update_leds(Adafruit_NeoPixel::Color(0xFF,0x00,0x90));

  // All done
  return;

}

void update_leds(uint32_t color)
{

  // Update the LED
  strip_internal_.fill(color);
  strip_internal_.show();

  strip_external_.fill(color);
  strip_external_.show();

  // All done
  return;

}
