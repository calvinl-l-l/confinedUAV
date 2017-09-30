#include "LED.h"

LED::LED()
{
	LED_L = Adafruit_NeoPixel(4, PIN_LED_L, NEO_GRB + NEO_KHZ800);
	LED_R = Adafruit_NeoPixel(4, PIN_LED_R, NEO_GRB + NEO_KHZ800);
	LED_M = Adafruit_NeoPixel(2, PIN_LED_M, NEO_GRB + NEO_KHZ800);
	pinMode(PIN_BUZZER, OUTPUT);
	digitalWrite(PIN_BUZZER, HIGH);	// mute the buzzer

	prev_mode = 'i';
}

void LED::init()
{
	LED_L.begin();
	LED_R.begin();
	LED_M.begin();
}

void LED::signalSwitcher(char sig)
{
	mode = sig;

	if (mode == 'm')
	{
		mode_manual();
		
		if (prev_mode != mode)
		{
			prev_mode = 'm';
			sigBuzzer(2, 3);
		}
	}
	else if (mode == 'a')
	{
		mode_auto();

		if (prev_mode != mode)
		{
			prev_mode = 'a';
			sigBuzzer(4, 2);
		}
	}
  else if (mode == 's')
  {
    mode_manual();
    prev_mode = 's';
  }
  else if (mode = 'i')
  {
    awesome_rainbow();
  }

}

void LED::awesome_rainbow()
{
	int i, j;

	// left and right LED
	for (j = 0; j<256; j++) {
		for (i = 0; i<4; i++) 
		{
			LED_L.setPixelColor(i, Wheel((i + j) & 255));
      LED_R.setPixelColor(i, Wheel((i + j) & 255));
		}

    LED_L.show();
    LED_R.show();
		
		LED_M.setPixelColor(0, Wheel((0 + j) & 255));
		LED_M.setPixelColor(1, Wheel((1 + j) & 255));
		
		LED_M.show();

		chThdSleepMilliseconds(5);
	}

	// mid LED

}


void LED::mode_auto()
{
	static int x = 0;

	// showing left and right indicator
	if (prev_mode != mode)
	{
		for (int i = 0; i < 4; i++)
		{
			LED_R.setPixelColor(i, LED_R.Color(255, 30, 30));
			LED_L.setPixelColor(i, LED_L.Color(255, 30, 30));
		}
		LED_L.show();
		LED_R.show();
	}

	// showing mid indicator
	x = 1 - x;
	LED_M.setPixelColor(x, LED_M.Color(255, 30, 30));
	x = 1 - x;
	LED_M.setPixelColor(x, 0);
	x = 1 - x;
	LED_M.show();

	chThdSleepSeconds(1);
}

void LED::mode_manual()
{
	if (prev_mode != mode)
	{
		for (int i = 0; i < 4; i++)
		{
			LED_R.setPixelColor(i, LED_R.Color(0, 255, 70));
			LED_L.setPixelColor(i, LED_L.Color(0, 255, 70));
		}
		LED_M.setPixelColor(0, LED_M.Color(0, 255, 70));
		LED_M.setPixelColor(1, LED_M.Color(0, 255, 70));

		LED_M.show();
		LED_L.show();
		LED_R.show();
	}
}


void LED::sigBuzzer(int n, int rate)
{
// n = beep n times
// rate: 1-3, 3 = faster rate
	int napTime;

	switch (rate)
	{
	case 1:
		napTime = 500;
		break;
	case 2:
		napTime = 300;
		break;
	case 3:
		napTime = 100;
		break;
	default:
		napTime = 300;
		break;
	}

	for (int i = 0; i < n; i++)
	{
		digitalWrite(PIN_BUZZER, LOW);
		chThdSleepMilliseconds(BUZZER_ONTIME);
		digitalWrite(PIN_BUZZER, HIGH);
		chThdSleepMilliseconds(napTime - BUZZER_ONTIME);
	}
}

uint32_t LED::Wheel(byte WheelPos) {
	
	Adafruit_NeoPixel strip;	// dummy strip
	WheelPos = 255 - WheelPos;
	
	if (WheelPos < 85) {
		return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
	}
	if (WheelPos < 170) {
		WheelPos -= 85;
		return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
	}
	WheelPos -= 170;
	return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
