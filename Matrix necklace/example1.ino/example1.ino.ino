
#include <MaxMatrix.h>
#include <avr/pgmspace.h>

int data = 2;
int load = 3;
int clock = 4;
MaxMatrix matrix(data, load, clock, 1);

void setup()
{
  matrix.init();
  matrix.setIntensity(8);
  Serial.begin(9600);
  matrix.setDot(0,0,1);
}

void loop()
{
  for(int i=0;i<8;i++){
    for(int j=0;j<8;j++){
      matrix.setDot(j,i,1);
      delay(100);
      matrix.clear();
    }
  }
}
