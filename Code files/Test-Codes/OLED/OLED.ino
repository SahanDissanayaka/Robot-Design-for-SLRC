#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define screen_width 128
#define screen_height 64
#define oled_reset -1
#define screen_address 0x3C

Adafruit_SSD1306 display(screen_width, screen_height, &Wire, oled_reset);

void display_line(String text, int column, int row, int text_size) {

  display.setTextSize(text_size);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(column, row);
  display.println(text);

  display.display();
}

void setup() {
  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, screen_address)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true) {}
  }

  display.display();
  delay(2000);

  display.clearDisplay();
  display_line("We are Groot", 0, 0,2);

  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

}
