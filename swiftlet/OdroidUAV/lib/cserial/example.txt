#include <iostream>
#include "cSerial.h"
#include <unistd.h>

using namespace std;

int a = 0;

int main()
{

  cSerial sp("/dev/ttyUSB0", 115200);
  sp.flush();

  while(1)
  {



      char b = (char) sp.Getchar();

      if (a)
      {
        sp.puts("$1234-0089+1#");
        a = 0;
       }
       else
       {
           sp.puts("$0234+1089-0#");
           a = 1;
       }
        cout << "Getting " << b << endl;

      int ms = 500;
      //usleep(ms * 1000);
  }

  return 0;
}
