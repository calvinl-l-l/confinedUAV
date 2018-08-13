// LED.h

#ifndef _LED_h
#define _LED_h

/*
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
*/

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ChNil.h>

#define NUM_REAR_PIXEL 40
#define NUM_FRONT_PIXEL 32

// pin assignments
#define PIN_LED_rear 46
#define PIN_LED_front 44

class LED
{
public:
	LED();
	void init();
	void signalSwitcher(char in);

private:
	Adafruit_NeoPixel LED_rear;
  Adafruit_NeoPixel LED_front;


	char prev_mode;
	char mode;

	void awesome_rainbow();
	void mode_manual();
	void mode_auto();
	uint32_t Wheel(byte WheelPos);
};


#endif
