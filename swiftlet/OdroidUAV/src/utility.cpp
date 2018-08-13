#include "utility.h"

void scan2pixelmap(vector<double> x, vector<double> y, double xc, double yc, int *map)
{
  int h2 = 25;
  int l2 = -25;
  int h1 = 5000;
  int l1 = -5000;


  int len = x.size();

  for (int i=0;i<len;i++)
  {
    float px = l2 + (x[i] - l1) * (h2 - l2) / (h1 - l1);
    float py = l2 + (y[i] - l1) * (h2 - l2) / (h1 - l1);

    if (px > h2)      px = h2;
    else if (px < l2) px = l2;
    if (py > h2)      py = h2;
    else if (py < l2) py = l2;

    map[(int) (round(py) + h2) * 50 + (int) round(px)+h2] = map[(int) (round(py) + h2) * 50 + (int) round(px)+h2] + 1;
  }

  float x0 = l2 + (xc - l1) * (h2 - l2) / (h1 - l1);
  float y0 = l2 + (yc - l1) * (h2 - l2) / (h1 - l1);

  map[(int) (round(y0) + h2) * 50 + (int) round(x0)+h2] = 999; // quad location

  x0 = l2 + (0 - l1) * (h2 - l2) / (h1 - l1);
  y0 = l2 + (0 - l1) * (h2 - l2) / (h1 - l1);

  map[(int) (round(y0) + h2) * 50 + (int) round(x0)+h2] = 9999; // (0,0)

}

int median(vector<long> in)
{
  int m;

  size_t size = in.size();

  sort(in.begin(), in.end());

  if (size % 2 == 0)
  {
    m = (in[size/2 - 1] + in[size/2]) /2;
  }
  else
  {
    m = in[size/2];
  }

  return m;
}


long val_remap(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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


//*****************************************************************************
// UI class
//*****************************************************************************
UI::UI()
{
    input = "";
}
