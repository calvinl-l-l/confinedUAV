#include <ChNil.h>
#include <ChNilSerial.h>
#include <ServoTimer1.h>  // Using timer1 on pwm removes servo jitter due to interrupts from rtos

#include "LED.h"


ChNilSerialClass ChNilSerial;
#define Serial ChNilSerial

#define PIN_WEBCAM_SERVO 9  // PIN for servo 

// Variables
ServoTimer1 webcam;
LED coolness;

char cmd = 'i'; // input command from Odroid

THD_WORKING_AREA(waThd_serial, 64);
THD_WORKING_AREA(waThd_servo, 64);
THD_WORKING_AREA(waThd_signal, 64);


THD_FUNCTION(thd_serial, arg) 
{
  (void)arg;
  while (TRUE) 
  {
    if (Serial.available()) cmd = Serial.read();
    chThdSleepMilliseconds(5);
  }
}


THD_FUNCTION(thd_servo, arg) 
{
  (void)arg;
  
  while (TRUE) 
  {
    static int pos = 30;
    static char dir = '+';
    
    if (pos > 110)     dir = '-';
    else if (pos < 50) dir = '+';

    if (cmd != 'i') webcam.write(pos);

    if (dir == '+')      pos += 10;
    else if (dir == '-') pos -= 10;
    chThdSleepMilliseconds(500);
  }
}


THD_FUNCTION(thd_signal, arg) 
{
  (void)arg;
  while (TRUE) 
  {
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

  webcam.attach(PIN_WEBCAM_SERVO);
  webcam.write(90); // init pos
  
  coolness.init();
    
  chFillStacks();
  chBegin();
}

void loop() {}  // idle thread
