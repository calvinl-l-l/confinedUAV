#include "utility.h"



long val_remap(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void signal_LED(int fd, int boundary, int mode)
{
  char msg[4];
  static char b = '1';
  static char f = '1';
  char tunnel_boundary;
  char flight_mode;

  if (boundary) tunnel_boundary = 'O';
  else tunnel_boundary = 'I';

  if (mode) flight_mode = 'A';
  else flight_mode = 'M';



  // send tunnel_boundary, then flight_mode
  if (b != tunnel_boundary || f != flight_mode)
  {
    snprintf(msg, 4, "<%c%c>", tunnel_boundary, flight_mode);
    serialPuts(fd, msg);
  }

  b = tunnel_boundary;
  f = flight_mode;
}

void signal_LED(int flag_auto_mode, int outside)
{
  // A B
  // 0 0 = manual
  // 0 1 = auto
  // 1 0 = low battery in auto
  // 1 1 = low battery in manual

  if (flag_auto_mode) // auto
  {
    digitalWrite(LED_LOGIC_A, HIGH);
    digitalWrite(LED_LOGIC_B, LOW);

    if (outside)
    {
      digitalWrite(LED_LOGIC_A, LOW);
      digitalWrite(LED_LOGIC_B, LOW);
    }
  }
  else if (!flag_auto_mode) // manual
  {
    digitalWrite(LED_LOGIC_A, LOW);
    digitalWrite(LED_LOGIC_B, LOW);

    if (outside)
    {
//      digitalWrite(LED_LOGIC_A, HIGH);
//      digitalWrite(LED_LOGIC_B, HIGH);
    }
  }

}
