//Set up accelerometer variables
float accBiasX, accBiasY, accBiasZ;
float accAngleX, accAngleY;
long accStdG;
double accRoll = 0;

//Set up gyroscope variables
float gyroBiasX, gyroBiasY, gyroBiasZ;
float gyroRateX, gyroRateY, gyroRateZ;
float gyroRoll = 0;
//double gyro_sensitivity = 70;  //From datasheet, depends on Scale, 2000DPS = 70, 500DPS = 17.5, 250DPS = 8.75. 

// Gyro Variables
int x;
int y;
int z;

void InitSensors() {
	int res;
	long tempAccZ, tempAccY;
	float tmpAccF;
	char tmpAccF_Str[10];
	delay(500);

	tempAccZ = 0;
	tempAccY = 0;
	tmpAccF = 0;

	for (int i = 0; i < 100; i++) {      // Takes 100 values to get more precision
		res = Acc.read(&x, &y, &z);
		if (z>0) {
			if (tempAccZ<z)
				tempAccZ = z + 1;
		}
		else {
			if (tempAccZ>z)
				tempAccZ = z + 1;
		}
		if (y>0) {
			if (tempAccY<y)
				tempAccY = y + 1;
		}
		else {
			if (tempAccY>y)
				tempAccY = y + 1;
		}
		delay(2);
	}
	accStdG = tempAccY*tempAccY + tempAccZ*tempAccZ;
	tmpAccF = sqrt(tempAccY*tempAccY + tempAccZ*tempAccZ);

	Serial.print("Avrage acceleration: ");
	Serial.print(tempAccY);
	Serial.print(", ");
	Serial.print(tempAccZ);
	Serial.print(", ");
	Serial.print(accStdG);
	Serial.print(", ");
	//dtostrf(tmpAccF*2.0*9.81/512, 3, 2, tmpAccF_Str);
	dtostrf(tmpAccF, 3, 2, tmpAccF_Str);
	Serial.println(tmpAccF_Str);

	// Calculate bias for the Gyro and accel, the values it gives when it's not moving
	// You have to keep the robot vertically and static for a few seconds
	//Gyr.initBasic(190, 2000);  //0,07
	Gyr.initBasic(190, 2000);  //0,0175

							   // Final bias values for every axis
	gyroBiasX = Gyr.dc_offsetX();
	gyroBiasY = Gyr.dc_offsetY();
	gyroBiasZ = Gyr.dc_offsetZ();

	//Get Starting Pitch and Roll
	res = Acc.read(&x, &y, &z);
	accRoll = (atan2(z, -y) + PI)*RAD_TO_DEG;

	if (accRoll <= 360 & accRoll >= 180) {
		accRoll = accRoll - 360;
	}
	Serial.print("Roll Inicial: ");
	Serial.println(InitialRoll);
	gyroRoll = accRoll;

}

void InitialValues() {
	//  Accelerometer   
	double InitialAngle = 0;
	double dGyro = 0;
	int res;

	for (int i = 0; i < 100; i++) {      // Takes 100 values to get more precision
		res = Acc.read(&x, &y, &z);
		accRoll = (atan2(z - accBiasZ, -(y - accBiasY)) + PI)*RAD_TO_DEG;

		Gyr.read(&x, &y, &z);
		if (y >= Gyr.noiseY() || y <= -Gyr.noiseY()) {
			gyroRateY = ((int)y - Gyr.dc_offsetY())*.07;
			//gyroRateY = ((int)y - Gyr.dc_offsetY())*.0175; 

			dGyro = gyroRateY * ((double)(micros() - timer) / 1000000);

			InitialAngle = 0.98* (InitialAngle + dGyro) + 0.02 * (accRoll);
			timer = micros();


		}
		delay(1);
	}
	InitialRoll = InitialAngle;
	Serial.print("Roll Inicial: ");
	Serial.println(InitialRoll);
}

// Roll from accelerometer
double getAccelRoll() {
	int res;
	long tmpVect;
	double a = 0;
	float f = 0;
	static double tmp_accRoll;
	char buf[64];
	res = Acc.read(&x, &y, &z);
	//sprintf(buf, "Acc x=%d, y=%d, d=%d\n", x, y, z);
	//Serial.print(buf);
	if (z > 200 || z < -200) {
		if (blockMotors == 0) {
			blockMotors = 1;
			Lcd.clear();
			Lcd.locate(0, 0);
			Lcd.print("motor block");
		}
	}
	else{
		if (blockMotors == 1) {
			Lcd.clear();
			blockMotors = 0;
		}
	}

	accRoll = (atan2(z, -(y)) + PI)*RAD_TO_DEG;   // Calculate the value of the angle 

	tmpVect = (long)z*(long)z + (long)y*(long)y;
	if (tmpVect>accStdG) {

		//Serial.print("accRoll=");
		//Serial.print(accRoll);
		//Serial.print(", tmp_accRoll=");
		//Serial.println(tmp_accRoll);
		accRoll = tmp_accRoll;
	}
	else {
		tmp_accRoll = accRoll;
	}

	if (accRoll <= 360 & accRoll >= 180) {
		accRoll = accRoll - 360;
	}

	// Serial.print(" AccRol: ");
	// Serial.println(accRoll);
	return accRoll;
}

// Angular velocity of Roll by gyroscope
double getDGyroRoll() {
	double dgyroRoll;
	Gyr.read(&x, &y, &z);     // Get values from gyro
	gyroRateY = -((int)y - gyroBiasY)*.07;

	dgyroRoll = gyroRateY * ((double)(micros() - timer) / 1000000);
	return dgyroRoll;
}





