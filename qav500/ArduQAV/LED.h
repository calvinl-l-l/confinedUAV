// LED.h

#ifndef _LED_h
#define _LED_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Adafruit_NeoPixel.h>
#include <ChNil.h>

// pin assignments
#define PIN_LED_L 6
#define PIN_LED_R 10
#define PIN_LED_M 5
#define PIN_BUZZER 8
#define BUZZER_ONTIME 50 // ms

class LED
{
public:
	LED();
	void init();
	void signalSwitcher(char in);

private:
	Adafruit_NeoPixel LED_L;
	Adafruit_NeoPixel LED_R;
	Adafruit_NeoPixel LED_M;

	char prev_mode;
	char mode;

	void awesome_rainbow();
	void mode_manual();
	void mode_auto();
	void sigBuzzer(int n, int rate);
	uint32_t Wheel(byte WheelPos);
};


#endif

