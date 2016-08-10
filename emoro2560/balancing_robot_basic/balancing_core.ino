#include "sensors.h"




float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void initBalancingCore() {
	uint16_t cnt;

	timer = micros();// Initialize timer

	Lcd.locate(0, 0);
	Lcd.print("calibrating");

	InitSensors();          // Initialize sensors
	initMot();              // Initialize motors
	delay(20);            // Wait until sensors are read
	InitialValues();        // Get the initial angle values
	RollAngle = InitialRoll;

	//INIT PID
	Input = getAccelRoll();
	Setpoint = -5.2;//Setpoint = -3.1;//Setpoint = -4.6;
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
		if (millis() - startScreenTimer >= 700) {
			startScreenTimer = millis();
			Lcd.clear();
			Lcd.locate(0, 0);
			Lcd.print("BalancingRobotV2 ");
			Lcd.locate(1, 0);
			Lcd.print("Start -> SW1");
			Lcd.locate(1, 6);
			if (startArrowState == 0)
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



void updateBalancingCore() {
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

	}
}

void Show(double a, double b) {			// Show data via serial port
	

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