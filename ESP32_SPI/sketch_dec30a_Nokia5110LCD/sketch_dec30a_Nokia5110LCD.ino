#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

/* Declare LCD object for SPI
 Adafruit_PCD8544(CLK,DIN,D/C,CE,RST); */
Adafruit_PCD8544 display = Adafruit_PCD8544(18, 23, 4, 15, 2);
int contrastValue = 60; // Default Contrast Value
const int adcPin = 34;
int adcValue = 0;

void setup()
{
  /* Initialize the Display*/
  display.begin();

  /* Change the contrast using the following API*/
  display.setContrast(contrastValue);

  /* Clear the buffer */
  display.clearDisplay();
  display.display();
  delay(1000);
  
  /* Now let us display some text */
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,1);
  display.setTextSize(2);
  display.println("|ESP32|");
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(22,20);
  display.println("|Nokia|");
  display.setCursor(22,32);
  display.println("|5110|");
  display.display();
  delay(2000);
}

void loop()
{
  /* You can implement your own display logic here*/  
}