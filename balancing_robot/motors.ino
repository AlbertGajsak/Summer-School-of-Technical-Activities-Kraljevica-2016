//motor A definitions
int bin1 = PWM_2;
int bin2 = PWM_1;
int pwmB = PWM_0;
//motor B definitions
int ain1 = PWM_3;
int ain2 = PWM_4;
int pwmA = PWM_5;

void initMot() {
	//motor A pinMode
	pinMode(ain1, OUTPUT);
	pinMode(ain2, OUTPUT);
	pinMode(pwmA, OUTPUT);
	//motor B pinMode
	pinMode(bin1, OUTPUT);
	pinMode(bin2, OUTPUT);
	pinMode(pwmB, OUTPUT);
	// Set direction to no-direction
	digitalWrite(ain1, HIGH);
	digitalWrite(ain2, HIGH);
	digitalWrite(bin1, HIGH);
	digitalWrite(bin2, HIGH);
}

void stopMot(void) {
	digitalWrite(ain1, HIGH);
	digitalWrite(ain2, HIGH);
	digitalWrite(bin1, HIGH);
	digitalWrite(bin2, HIGH);
	digitalWrite(pwmA, LOW);
	digitalWrite(pwmB, LOW);
}
void MotorControl(double out) {
	// Sets direction:
	if (out < 0) {              // forward
		digitalWrite(ain1, HIGH);
		digitalWrite(ain2, LOW);
		digitalWrite(bin1, LOW);
		digitalWrite(bin2, HIGH);
	}
	else {                     // backward
		digitalWrite(ain1, LOW);
		digitalWrite(ain2, HIGH);
		digitalWrite(bin1, HIGH);
		digitalWrite(bin2, LOW);
	}
	byte vel = abs(out);    // Absolute value of velocity
	if (Strana>0) {
		//velL = 0;
		velL = vel*Strana;
		velR = vel;
	}
	else if (Strana<0) {
		//velR = 0;
		velR = vel*Strana*-1;
		velL = vel;
	}
	else {
		velR = vel;
		velL = vel;
	}

	// Checks velocity fits the max ouptut range
	if (vel<0)
		vel = 0;
	
	if (vel > 255)
		vel = 255;

	// Writes the PWM 
	if (blockMotors == 0) {//
		analogWrite(pwmA, velL);
		analogWrite(pwmB, velR);
	}
	else {
		analogWrite(pwmA, 0);
		analogWrite(pwmB, 0);
	}
}