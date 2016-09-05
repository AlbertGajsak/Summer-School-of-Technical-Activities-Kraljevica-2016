#include <EEPROMex.h>

bool blockMotors = 0;
byte velL = 0;
byte velR = 0;

float turnOffset = 0.4;

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}


#include <Kalman.h>
#include <PID_v1.h>

// Rutine for repeat part of the code every X miliseconds
static long _lasttime1;
static long _lasttime2;
static long _lasttime3;


//Set up a timer Variable
uint32_t timer;
uint8_t pidMode;

double InputRoll;    // Roll angle value
double InitialRoll;  // Roll initial angle

					 //Define Variables we'll be connecting to
double Setpoint = 0, Input, Output, Strana = 0;

//Define the aggressive and conservative Tuning Parameters
//double aggKp=4, aggKi=0.2, aggKd=1;
//double aggKp=6.5, aggKi=75, aggKd=0.08;

//double aggKp = 30, aggKi = 5, aggKd = 0;


//double consKp=30, consKi=50, consKd=1;
//double consKp = 32, consKi = 500, consKd = 0.01;
//double consKp = 10, consKi = 50, consKd = 0.1;

//double consKp = 30, consKi = 900, consKd = 1;

//double consKp = 60, consKi = 900, consKd = 1;
//double consKp = 10, consKi = 0, consKd = 0.1;
//double consKp = 15, consKi = 854, consKd = 0.8;
double consKp = 15, consKi = 500, consKd = 0.8;

double consKpS = 60, consKiS = 900, consKdS = 1;

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);//Specify the links and initial tuning parameters

static uint8_t counter;
uint8_t overSpeedCntF, overSpeedCntB;

double RollAngle = 0; // Roll angle variable

void setup() {
  //EEPROM.writeDouble(0,0);
	uint16_t cnt;
	Serial.begin(9600);	
	//Serial1.begin(9600);

	timer = micros();// Initialize timer
	
	Lcd.locate(0, 0);
	Lcd.print("calibrating");

	InitSensors();          // Initialize sensors
	initMot();              // Initialize motors
	delay(20);            // Wait until sensors are read
	InitialValues();        // Get the initial angle values
	RollAngle = InitialRoll;
	//EEPROM.writeDouble(0, 3.0);
	//INIT PID
	Input = getAccelRoll();
	Setpoint = EEPROM.readDouble(0);
	Serial.println(Setpoint);
	if (Setpoint == 0)	Serial.println("ERROR: no value in EEPROM");
	myPID.SetTunings(consKp, consKi, consKd);
	myPID.SetOutputLimits(-255, 255);
	myPID.SetMode(AUTOMATIC);
	myPID.SetSampleTime(10);

	delay(500);			//wait half a second
	
	unsigned long startScreenTimer = millis();	
	bool startArrowState = 0;
	//Hello world sound
	Lcd.locate(0, 0);
	Lcd.print("BalancingRobotV2 ");
	tone(BUZ_BUILTIN, 500, 20);
	delay(40);
	tone(BUZ_BUILTIN, 1000, 20);
	delay(40);
	tone(BUZ_BUILTIN, 1500, 20);


	
	while (ReadSwitch(SW_1) == 0) {
		if (millis()-startScreenTimer >= 700) {
			startScreenTimer = millis();
			Lcd.clear();
			Lcd.locate(0, 0);
			Lcd.print("BalancingRobotV2 ");
			Lcd.locate(1, 0);
			Lcd.print("Start -> SW1");
			Lcd.locate(1, 6);
			if(startArrowState==0)
				Lcd.print("->");
			else
				Lcd.print("-->");
			Lcd.locate(1, 9);
			Lcd.print("SW1");
			startArrowState = !startArrowState;
		}

	} //start the program when SW_1 is pressed

	_lasttime1 = millis();
	_lasttime2 = millis();
	_lasttime3 = millis();
	pidMode = 0;
	counter = 0;

	delay(500);

	cnt = 100;
	while (cnt > 0) {
		RollAngle = 0.98 * (RollAngle + getDGyroRoll()) + 0.02 * (getAccelRoll());
		timer = micros();    // Reset the timer
		delay(1);
		cnt--;
	}

	MotorControl(0);
	cnt = 0;
	while (cnt > 0) {
		RollAngle = 0.98 * (RollAngle + getDGyroRoll()) + 0.02 * (getAccelRoll());
		timer = micros();    // Reset the timer
		delay(1);
		cnt--;
	}
	overSpeedCntF = 0;
	overSpeedCntB = 0;

	Lcd.clear();
}



int stopSW = 1;

int avrSpeed[100];
int avrSpe;
unsigned int ind = 0;
unsigned int k;

int debugScreenNum = 0, maxScreenNum = 2;

void loop() {
	int x, y, z;
	char buf[32];
	double gyro, accel;


	if ((uint16_t)((uint16_t)millis() - _lasttime2) >= (10)) {
		_lasttime2 += 10;
		

		gyro = RollAngle + getDGyroRoll();
		accel = getAccelRoll();
		RollAngle = 0.98 * gyro + 0.02 * accel;
		if (stopSW == 0) {
			Serial.println("Robot stopped");
		}
		timer = micros();    // Reset the timer
	}
	if ((uint16_t)((uint16_t)millis() - _lasttime3) >= (10)) {      // Exetutes this part of the code every 10 miliseconds -> 100Hz
		_lasttime3 += 10;

		if (stopSW == 0)
			RollAngle = -7.62;

		Input = RollAngle;

		double gap = abs(Setpoint - Input); //distance away from setpoint
		gap = 0;
		if (gap < 3) //was 2 i set 3
		{ //we're close to setpoint, use conservative tuning parameters
			if (pidMode != 1) {
				//pidMode = 1;
				myPID.SetTunings(consKp, consKi, consKd);
			}
		}
		//else if (pidMode != 2)
		//{
		//	if (pidMode != 2) {
		//		//we're far from setpoint, use aggressive tuning parameters
		//		pidMode = 2;
		//		myPID.SetTunings(aggKp, aggKi, aggKd);
		//	}
		//}
		myPID.Compute();

		avrSpeed[ind] = Output;
		ind++;
		if (ind == 10)
			ind = 0;

		/* if(avrSpe<-40)
		EmoroServo.write(SERVO_0, 2300);
		else if(avrSpe>40)
		EmoroServo.write(SERVO_0, 1000);
		else
		EmoroServo.write(SERVO_0, 1700);

		*/

		if (stopSW == 1) {
			if (Output > 0) {
				//backwardMot((unsigned char)Output);
				//backwardMot(Output);
				MotorControl(Output);
			}
			else if (Output < 0) {
				//forwardMot(abs(Output));
				MotorControl(Output);
			}
			else {
				stopMot();
			}
		}
	}

	if ((uint16_t)((uint16_t)millis() - _lasttime1) >= (100)) {
		_lasttime1 += 100;


		avrSpe = 0;
		for (k = 0; k < 10; k++) {
			avrSpe += avrSpeed[k];
			if (k > 0) {
				if ((avrSpeed[k - 1] > 0 && avrSpeed[k] < 0) || (avrSpeed[k - 1] < 0 && avrSpeed[k] > 0)) {
					avrSpe = 0;
				}
			}
		}
		avrSpe = avrSpe / 10;

		//      if(avrSpe<-10)
		//        Setpoint=-7.55;
		//     else if(avrSpe<-5)
		//        Setpoint=-7.55;
		//     else if(avrSpe>10)
		//        Setpoint=-7.52;
		//     else if(avrSpe>5)
		//        Setpoint=-7.52;
		//     else
		//        Setpoint=-7.52;
		//getGyroRoll();

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
		EEPROM.writeDouble(0, Setpoint);
		while (ReadSwitch(SW_1) != 0);
	}
	else if (ReadSwitch(SW_2) == 1) {
		Setpoint -= 0.1;
		EEPROM.writeDouble(0, Setpoint);
		while (ReadSwitch(SW_2) != 0);
	}
	ReceiveData();      // Checks Serial for incoming data

}


//bluetooth control:
void ReceiveData() {

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
			EEPROM.writeDouble(0, Setpoint);
			break;
		case 's':
			Setpoint -= 0.1;
			EEPROM.writeDouble(0, Setpoint);
			break;

		case '1':
			tone(BUZ_BUILTIN, 1000, 20);
			break;


			break;
		case 'R':
			Strana += turnOffset;
			Setpoint += 0.5;
			//Setpoint += 3;
			break;
		case 'r':
			Strana -= turnOffset;
			Setpoint -= 0.5;
			//Setpoint -= 3;
			break;

		case 'L':
			Strana -= turnOffset;
			//Setpoint += 3;
			break;
		case 'l':
			Strana += turnOffset;
			//Setpoint -= 3;
			break;

		case 'T':
			turnOffset += 0.01;
			break;
		case 't':
			turnOffset -= 0.01;
			break;
		
		case 'X':
			motors(-127, -127);
			delay(200);
			break;

			//      case 's':
			//        //InitialRoll = RollAngle;
			//        Show(RollAngle, 0);
			//        break;
		}
		myPID.SetTunings(consKp, consKi, consKd);
	}

}

// Show data via serial port
void Show(double a, double b) {

	Serial.print("DATOS: ");
	Serial.print("Roll: ");
	Serial.print(a);
	Serial.print("Acc roll: ");
	Serial.print(b);


}
void ShowAcc(double a, double b, double c, double d) {

	Serial.print("ACC Data x=");
	Serial.print(a);
	Serial.print(" y=");
	Serial.print(b);
	Serial.print(" z=");
	Serial.print(c);
	Serial.print(" Accdeg=");
	Serial.println(d);

}
void ShowTwo(double a, double b) {

	Serial.print("Gyr dx=");
	Serial.print(a);
	Serial.print(" Accdeg=");
	Serial.println(b);

}

