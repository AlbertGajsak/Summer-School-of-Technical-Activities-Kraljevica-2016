#include <Kalman.h>
#include <PID_v1.h>
#include <EEPROMex.h>
//--------------------------------------------------------------------------------------------------------------------------------
byte firstStart=0;

long _lasttime1, _lasttime2, _lasttime3;		//used for counting with millis()
uint32_t timer;		//Set up a timer Variable
uint8_t pidMode;
double InputRoll;    // Roll angle value
double InitialRoll;  // Roll initial angle
double Setpoint, Input, Output, Strana = 0;
double consKp, consKi, consKd;
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
  if(EEPROM.readByte(0)!=1){
    EEPROM.writeByte(0, 1);
    consKp = 15;
    consKi = 854;
    consKd = 0.8;
    Setpoint = -5.2;
    EEPROM.writeDouble(1,Setpoint);
    EEPROM.writeDouble(2,consKp);
    EEPROM.writeDouble(3,consKi);
    EEPROM.writeDouble(4,consKd);
  }
  
    Setpoint = EEPROM.readDouble(1);
    consKp = EEPROM.readDouble(2);
    consKi = EEPROM.readDouble(3);
    consKd = EEPROM.readDouble(4);    
    Serial.println(consKp);
    Serial.println(consKi);
    Serial.println(consKd);
  
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

				//consKp = analogRead(ADC_0);
				//consKp = consKpS + mapfloat(analogRead(ADC_0), 0, 1024, -60, 60);
				//consKp += mapfloat(analogRead(ADC_0), 0, 1024, -60, 60);
				//consKp += mapfloat(analogRead(ADC_0), 0, 1024, -60, 60);


				//-------------------------------
				break;


			case 1:
				//-------------------------------
				Lcd.locate(0, 0);
				Lcd.print("T");
				Lcd.locate(0, 1);
				Lcd.print(turnOffset);

				/*
				Lcd.locate(0, 9);
				Lcd.print("I");
				Lcd.locate(0, 10);
				Lcd.print(consKi);
				*/

				Lcd.locate(1, 0);
				Lcd.print(velL);

				Lcd.locate(1, 8);
				Lcd.print(velR);
				//-------------------------------
				break;
			}
		}
		if (counter < 50)
			counter += 1;
		else if (counter == 50) {
			counter += 1;
			//EmoroServo.write(SERVO_0, 2100);
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
    EEPROM.writeDouble(1,Setpoint);
		while (ReadSwitch(SW_1) != 0);
	}
	else if (ReadSwitch(SW_2) == 1) {
		Setpoint -= 0.1;
		EEPROM.writeDouble(1,Setpoint);
		while (ReadSwitch(SW_2) != 0);
	}
	receiveData();      // Checks Serial for incoming data

}


//bluetooth control:
void receiveData() {

	if (Serial1.available()) {
		char a = Serial1.read();
		switch (a) {
		case 'P':
			consKp += 1;
      EEPROM.writeDouble(2,consKp);
			break;
		case 'p':
			consKp -= 1;
      EEPROM.writeDouble(2,consKp);
			break;
		case 'I':
			consKi += 10;
      EEPROM.writeDouble(3,consKi);
			break;
		case 'i':
			consKi -= 10;
			EEPROM.writeDouble(3,consKi);
			break;
		case 'D':
			consKd += 0.1;
			EEPROM.writeDouble(4,consKd);
			break;
		case 'd':
			consKd -= 0.1;
			EEPROM.writeDouble(4,consKd);
			break;

		case 'F':


   
			Setpoint += 1;
			//Setpoint += 4;
			break;
		case 'f':
			Setpoint -= 1;
			//Setpoint -= 4;
			break;

		case 'B':
			Setpoint -= 1;
			//Setpoint -= 4;
			break;
		case 'b':
			Setpoint += 1;
			//Setpoint += 4;
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
			//Setpoint += 1;
			//Setpoint += 3;
			break;
		case 'r':
			Strana -= turnOffset;
			//Setpoint -= 1;
			Setpoint -= 1;
			//Setpoint -= 3;
			break;

		case 'L':
			Strana -= turnOffset;
			//Setpoint -= 1;
			Setpoint -= 1;
			//Setpoint += 3;
			break;
		case 'l':
			Strana += turnOffset;
			Setpoint += 1;
			//Setpoint -= 1;
			//Setpoint -= 3;
			break;

		case 'T':
			turnOffset += 0.01;
			break;
		case 't':
			turnOffset -= 0.01;
			break;

			//      case 's':
			//        //InitialRoll = RollAngle;
			//        Show(RollAngle, 0);
			//        break;
		}
		myPID.SetTunings(consKp, consKi, consKd);
	}

}



