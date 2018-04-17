// Drive.c
//
// Brandon Coll
//
// Code for basic robot drive functions.

float avgEncoders;
int drivePIDOutput;
bool driveDone = false;
bool turnDone = false;
int max_speed = 127;


void resetEncoders() {
	SensorValue(leftEncoder) = 0;
	SensorValue(rightEncoder) = 0;
}

float getGyro(){
	return (SensorValue(leftEncoder) - SensorValue(rightEncoder)) * 0.239;
}

float getAvgEncoder(){
	return (getRightEncoder() + getLeftEncoder()) /2 ;
}

// Drives the left drive at the given voltage.
void leftDrive(int voltage){
	motor(LRD) = motor(LRR) = motor(LFD) = motor(LFR) = removeDeadband(voltage);
}

//Drives the right drive at the given voltage.
void rightDrive(int voltage){
	motor(RRD) = motor(RRR) = motor(RFD) = motor(RFR) = removeDeadband(voltage);
}

void CurvatureDrive(int ySpeed, int zRotation){
	ySpeed = clamp(ySpeed, 127);
	zRotation = clamp(zRotation, 127);

	double leftOutput = ySpeed - zRotation;
	double rightOutput = ySpeed + zRotation;
	double maxInputMag = max(abs(ySpeed), abs(zRotation));
	if (maxInputMag > 127) {
		leftOutput /= maxInputMag;
		rightOutput /= maxInputMag;
	}

	leftDrive(leftOutput * max_speed);
	rightDrive(rightOutput * max_speed);
}

//ARCADE DRIVE SYSTEM
void arcadeDrive(){
	int throttle = removeDeadband(vexRT(Ch3));
	int turnThrottle = removeDeadband(vexRT(Ch1));
	if(vexRT(Btn5D)){
		leftDrive(throttle + turnThrottle / 2);
		rightDrive(throttle - turnThrottle / 2);
	}
	else{
		leftDrive(throttle + turnThrottle);
		rightDrive(throttle - turnThrottle);
	}
}

//TANK DRIVE SYSTEM
void tankDrive(){
	leftDrive(vexRT(Ch3));
	rightDrive(vexRT(Ch2));
}

void autoDrive(int voltage) {
	int turn = 0;
	if(abs(voltage) > 15){
		turn = getGyro() * 4;
	}
	//motor(LD1) = motor(LD2) = motor(LD3) = voltage - turn;
	//motor(RD1)  = motor(RD2) = motor(RD3) = voltage + turn;
}
