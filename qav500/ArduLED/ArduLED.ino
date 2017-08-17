/*
 * Program to control RGB LED on QAV500
 * 
 * Using Adafruit's NeoPixel library for controlling the RGB LED
*/

#include <Adafruit_NeoPixel.h>

#define LED_pin 6
#define wait 2

Adafruit_NeoPixel pix = Adafruit_NeoPixel(16, LED_pin, NEO_GRB + NEO_KHZ800);

char prev_mode;

void setup() 
{
  pix.begin();
  Serial.begin(9600);
}

void loop() 
{
  int b0 = analogRead(A2);
  int b1 = analogRead(A1);

  if (b0 > 200) b0 = 1;
  else          b0 = 0;
  if (b1 > 200) b1 = 2;
  else          b1 = 0;

  int b = b0 + b1;

  Serial.print(b0);
  Serial.print(' ');
  Serial.println(b1);
  
  switch (b)
  {
    case 0: // manual mode
      mode_manual();
      prev_mode = 'm';
      break;
    
    case 1: // auto mode
      mode_auto();  
      prev_mode = 'a';
      break;

    case 2: // low battery in auto mode
      low_battery_a();
      break;

    case 3:
      low_battery_m();
      break;
      
    default:
      mode_manual();
      break;
  }
  
  pix.show();
}


void mode_auto()
{
   for (int i=0; i<16; i++)
   {
    pix.setPixelColor(i, pix.Color(255, 10, 10)); 
   }
}


void mode_manual()
{
  for (int i=0; i<16; i++)
  {
    pix.setPixelColor(i, pix.Color(0, 255, 128)); 
  }
}

void low_battery_a()
{
  for (int i=256; i >50; i--){
    for (int j=0; j < 16; j++){
      pix.setPixelColor(j, pix.Color(i, 10 - 10*i/255, 10 - 10*i/255));
    }
    pix.show();
    delay(wait);
  }
  for (int i=50; i < 256; i++){
    for (int j=0; j < 16; j++){
      pix.setPixelColor(j, pix.Color(i, 10*i/255, 10*i/255));
    }
    pix.show();
    delay(wait);
  }
}

void low_battery_m()
{
  for (int i=256; i >50; i--){
    for (int j=0; j < 16; j++){
      pix.setPixelColor(j, pix.Color(0, i, 128 - 128*i/255));
    }
    pix.show();
    delay(wait);
  }
  for (int i=50; i < 256; i++){
    for (int j=0; j < 16; j++){
      pix.setPixelColor(j, pix.Color(0, i, 128*i/255));
    }
    pix.show();
    delay(wait);
  }
}
