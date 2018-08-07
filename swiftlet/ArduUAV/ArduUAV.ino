#include <ChNil.h>
#include <ChNilSerial.h>
#include "LED.h"



ChNilSerialClass ChNilSerial;
//#define Serial ChNilSerial

// pin assignments
#define PIN_LED_rear 46

// Variables

LED coolness;

char cmd = 'i'; // input command from Odroid

THD_WORKING_AREA(waThd_serial, 64);
THD_WORKING_AREA(waThd_servo, 64);
THD_WORKING_AREA(waThd_signal, 64);

SEMAPHORE_DECL(sem, 0);

THD_FUNCTION(thd_serial, arg)
{
  (void)arg;
  while (TRUE)
  {
    if (Serial.available()) cmd = Serial.read();
    if (Serial1.available()) cmd = Serial1.read();
    
    //Serial.print(cmd);
    chSemSignal(&sem);
    chThdSleepMilliseconds(5);
  }
}


THD_FUNCTION(thd_servo, arg)
{
  (void)arg;

  while (TRUE)
  {

    chSemWait(&sem);


    chThdSleepMilliseconds(500);
  }
}


THD_FUNCTION(thd_signal, arg)
{
  (void)arg;
  while (TRUE)
  {
    chSemWait(&sem);

    coolness.signalSwitcher(cmd);
    chThdSleepMilliseconds(50);
  }
}

THD_TABLE_BEGIN
  THD_TABLE_ENTRY(waThd_serial, NULL, thd_serial, NULL)
  THD_TABLE_ENTRY(waThd_servo, NULL, thd_servo, NULL)
  THD_TABLE_ENTRY(waThd_signal, NULL, thd_signal, NULL)
THD_TABLE_END

void setup()
{
  Serial.begin(9600);
  Serial1.begin(57600);

  coolness.init();

  chFillStacks();
  chBegin();
}

void loop() {}  // idle thread
