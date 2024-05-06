
// neopixel setup
int led_pin_     = 26;  // 16:internal, 26:external
uint16_t n_leds_ = 1;

// neopixel objects
Adafruit_NeoPixel strip_;

void init_leds()
{

  // Start the LED strips
  strip_.setPin(led_pin_);
  strip_.updateLength(n_leds_);
  strip_.updateType(NEO_GRB + NEO_KHZ800);

  // Start strip
  strip_.begin();
  strip_.setBrightness(255);

  // Flash LEDs
  update_leds(Adafruit_NeoPixel::Color(0xFF,0x00,0x00)); delay(1000);
  update_leds(Adafruit_NeoPixel::Color(0x00,0xFF,0x00)); delay(1000);
  update_leds(Adafruit_NeoPixel::Color(0x00,0x00,0xFF)); delay(1000);

  // All done
  return;

}

void update_leds(uint32_t color)
{

  // Update the LED
  strip_.fill(color);
  strip_.show();

  // All done
  return;

}
