#include <Arduino.h>
#ifndef SENSOR_H
#define SENSOR_H
const int TG = 31;      const int PG = 30;
const int T1 = 32;      const int P1 = 29;
const int T2 = 33;      const int P2 = 28;
const int T3 = 34;      const int P3 = 27;
const int T4 = 35;      const int P4 = 26;
const int cb1 = 25; 
int In_SenSor()
{
  int ss1  = digitalRead(T4);
  int ss2  = digitalRead(T3);
  int ss3  = digitalRead(T2);
  int ss4  = digitalRead(T1);
  int ss5  = digitalRead(TG);
  int ss6  = digitalRead(PG);
  int ss7  = digitalRead(P1);
  int ss8  = digitalRead(P2);
  int ss9  = digitalRead(P3);
  int ss10 = digitalRead(P4);

  // CASE 0: GIỮA LINE
  if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 0 && ss6 == 0     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)||
     (ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 0 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)||
     (ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 0     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)) {return 0;}

  // LỆCH PHẢI (Giá trị dương)
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 0     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return 1;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 0     && ss7 == 0 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return 2;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 0 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return 3;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 0 && ss8 == 0 && ss9 == 1 && ss10 == 1)){return 4;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 0 && ss9 == 1 && ss10 == 1)){return 5;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 0 && ss9 == 0 && ss10 == 1)){return 6;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 0 && ss10 == 1)){return 7;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 0 && ss10 == 0)){return 8;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 0)){return 9;}

  // LỆCH TRÁI (Giá trị âm)
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 0 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -1;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 0     && ss5 == 0 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -2;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 0     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -3;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 0 && ss4 == 0     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -4;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 0 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -5;}
  else if((ss1 == 1 && ss2 == 0 && ss3 == 0 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -6;}
  else if((ss1 == 1 && ss2 == 0 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -7;}
  else if((ss1 == 0 && ss2 == 0 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -8;}
  else if((ss1 == 0 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -9;}
  //  CASE 14: VẠCH TRẠM/DỪNG
  else if((ss1 == 1 && ss2 == 1 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0)||
          (ss1 == 1 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0)||
          (ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0)||
        //(ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 0 && ss9 == 0 && ss10 == 0)||
          (ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0)||
          (ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0)){return 14;}

else if((ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)||
        (ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 1 && ss9 == 1 && ss10 == 1)||
        (ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)||
        (ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)||
        (ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return 15;}

  else if(ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 0     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 0){return 16;}
  else if(ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1){return 13;}
  else if(ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0){return 12;}

}
#endif