/*
  balancing_robot_basic - basic code for a self balancing robot developed on the Summer School of Technical Activities Kraljevica 2016
  Created by Albert Gajšak, Paolo Zenzerović, Boris Ćurko
  August 2016
  Released into the public domain.
*/                     

//include necessary librarys
#include <Kalman.h> 
#include <PID_v1.h>
 
//----------------declaring initial variables and objects necessary for balancing------------------------------------------------
long _lasttime1, _lasttime2, _lasttime3;		//used for counting with millis()
uint32_t timer;		//Set up a timer Variable
uint8_t pidMode;
double InputRoll;    // Roll angle value
double InitialRoll;  // Roll initial angle
double Setpoint, Input, Output, Strana = 0;
double consKp = 15, consKi = 854, consKd = 0.8;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);//Specify the links and initial tuning parameters
uint8_t counter;
uint8_t overSpeedCntF, overSpeedCntB;
double RollAngle = 0; // Roll angle variable

int avrSpeed[100];
int avrSpe;
unsigned int ind = 0;
unsigned int k;

int stopSW = 1;

bool blockMotors = 0;
byte velL = 0;
byte velR = 0;
float turnOffset = 0.4;
//--------------------------------------------------------------------------------------------------------------------------------

void setup() {
	Serial.begin(9600);
	initBalancingCore();
}

int debugScreenNum = 0, maxScreenNum = 2;

void loop() {
	updateBalancingCore();

		if (blockMotors != 1) {
			switch (debugScreenNum) {
			case 0:
				//-------------------------------
				Lcd.locate(0, 0);
				Lcd.print("P");
				Lcd.locate(0, 1);
				Lcd.print(consKp);

				Lcd.locate(0, 9);
				Lcd.print("I");
				Lcd.locate(0, 10);
				Lcd.print(consKi);

				Lcd.locate(1, 0);
				Lcd.print("D");
				Lcd.locate(1, 1);
				Lcd.print(consKd);

				Lcd.locate(1, 8);
				Lcd.print(Setpoint);
				//-------------------------------
				break;
        
			case 1:
				//-------------------------------
				Lcd.locate(0, 0);
				Lcd.print("T");
				Lcd.locate(0, 1);
				Lcd.print(turnOffset);

				Lcd.locate(1, 0);
				Lcd.print(velL);

				Lcd.locate(1, 8);
				Lcd.print(velR);
				//-------------------------------
				break;
			}
		}

	if (ReadSwitch(SW_3) == 1) {
		stopSW = 0;
		stopMot();
	}
	else if (ReadSwitch(SW_4) == 1) {
		stopSW = 1;
		debugScreenNum++;
		if (debugScreenNum >= maxScreenNum)
			debugScreenNum = 0;
		Lcd.clear();
	}
	else if (ReadSwitch(SW_1) == 1) {
		Setpoint += 0.1;
		while (ReadSwitch(SW_1) != 0);
	}
	else if (ReadSwitch(SW_2) == 1) {
		Setpoint -= 0.1;
		while (ReadSwitch(SW_2) != 0);
	}
	receiveData();      // check the Serial port (bluetooth) for incoming data
}

//bluetooth receive routine (for remote control over smartphone)
void receiveData() {
	if (Serial1.available()) {
		char a = Serial1.read();
		switch (a) {
		case 'P':
			consKp += 1;
			break;
		case 'p':
			consKp -= 1;
			break;
		case 'I':
			consKi += 10;
			break;
		case 'i':
			consKi -= 10;
			break;
		case 'D':
			consKd += 0.1;
			break;
		case 'd':
			consKd -= 0.1;
			break;

		case 'F':
			Setpoint += 1;
			break;
		case 'f':
			Setpoint -= 1;
			break;

		case 'B':
			Setpoint -= 1;
			break;
		case 'b':
			Setpoint += 1;
			break;

		case 'S':
			Setpoint += 0.1;
			break;
		case 's':
			Setpoint -= 0.1;
			break;

		case '1':
			tone(BUZ_BUILTIN, 1000, 20);
			break;
      
			break;
		case 'R':
			Strana += turnOffset;
			Setpoint += 1;
			break;
		case 'r':
			Strana -= turnOffset;
			Setpoint -= 1;
			break;

		case 'L':
			Strana -= turnOffset;
			Setpoint -= 1;
			break;
		case 'l':
			Strana += turnOffset;
			Setpoint += 1;
			break;

		case 'T':
			turnOffset += 0.01;
			break;
		case 't':
			turnOffset -= 0.01;
			break;
		}
		myPID.SetTunings(consKp, consKi, consKd);
	}
}



