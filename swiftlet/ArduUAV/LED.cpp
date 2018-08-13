#include "LED.h"

LED::LED()
{
	LED_rear = Adafruit_NeoPixel(40, PIN_LED_rear, NEO_GRB + NEO_KHZ800);
  LED_front = Adafruit_NeoPixel(32, PIN_LED_rear, NEO_GRB + NEO_KHZ800);
	prev_mode = 'i';
}

void LED::init()
{
	LED_rear.begin();
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

		}
	}
	else if (mode == 'a')
	{
		mode_auto();

		if (prev_mode != mode)
		{
			prev_mode = 'a';

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
		for (i = 0; i<NUM_REAR_PIXEL; i++)
		{
			LED_rear.setPixelColor(i, Wheel((i + j) & 255));

		}

    for (i = 0; i<NUM_REAR_PIXEL; i++)
    {
      LED_front.setPixelColor(i, Wheel((i + j) & 255));

    }

    LED_front.show();
    LED_rear.show();


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
		for (int i = 0; i < NUM_REAR_PIXEL; i++)
		{
			LED_rear.setPixelColor(i, LED_rear.Color(255, 30, 30));
		}

    for (int i = 0; i < NUM_REAR_PIXEL; i++)
    {
      LED_front.setPixelColor(i, LED_rear.Color(0, 255, 70));
    }

    LED_front.show();
		LED_rear.show();
	}


	chThdSleepSeconds(1);
}

void LED::mode_manual()
{
	if (prev_mode != mode)
	{
		for (int i = 0; i < NUM_REAR_PIXEL; i++)
		{
			LED_rear.setPixelColor(i, LED_rear.Color(0, 255, 70));
		}
    for (int i = 0; i < NUM_REAR_PIXEL; i++)
    {
      LED_front.setPixelColor(i, LED_rear.Color(255, 255, 255));
    }

    LED_front.show();
		LED_rear.show();

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
