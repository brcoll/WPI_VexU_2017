//Drive control vars
bool isDriving = false;
bool isTurning = false;
bool isWall = false;
int wallPower = 100;
float linearDistance = 0;
float turnAng = 0;
int maxspeed = MAX_VOLTAGE;

//TIME FOR US TO GIVE UP
int stuckTimeout = 1000;
bool skills = false;

int driveErrorThreshold = 1;
int turnErrorThreshold = 1;

int Tight_driveErrorThreshold = 1;
int Tight_turnErrorThreshold = 1;

int DGAF_driveErrorThreshold = 2;
int DGAF_turnErrorThreshold = 4;


//Linear variables
float disterror, differror, distintegral, diffintegral,  distspeed, diffspeed, direction = 0;
float distderivative, diffderivative, prevdisterror, prevdifferror = 0;
float disterror_Stuck, distderivative_Stuck, prevdisterror_Stuck =0;
bool hitWall, hitSomething, goingBack = false, wallForward = false;
int lastLatched, lastLatched_Stuck, startingTime, wallTime = 0;

//LINEAR DRIVE GAINS
float distP = 14;
float distI = 0.25;
float distD = 125;
//STEADY DRIVE GAINS
float diffP = 8;
float diffI = 0.05;
float diffD = 0.2;
//TURN GAINS
float turnP = 10;
float turnI = 0.3;
float turnD = 30; // OLD 80

////Return value of left encoder in inches
//float getLeftEncoder() {
//	return ((float)SensorValue(leftEncoder) / 360.0) * 3.25 * PI;
//}
////Return value of right encoder in inches
//float getRightEncoder() {
//	return ((float)SensorValue(rightEncoder) / 360.0) * 3.25 * PI;
//}
////Return average value of encoders in inches
//float getAvgEncoder(){
//	return (getRightEncoder() + getLeftEncoder()) /2 ;
//}
////Return angle from difference in encoders
//float getGyro(){
//	return (SensorValue(leftEncoder) - SensorValue(rightEncoder)) / 6.74;
//}
//Reset both encoders to 0
//void resetEncoders() {
//	SensorValue(leftEncoder) = 0;
//	SensorValue(rightEncoder) = 0;
//}
//Turns robot at specific voltage
//void autoTurn(int voltage) {
//	motor(LD1) = motor(LD2) = motor(LD3) = voltage;
	//motor(RD1)  = motor(RD2) = motor(RD3) = -voltage;
//}

void initPID(bool tight = false){
	resetEncoders();
	prevdisterror = 0;
	prevdifferror = 0;
	distintegral = 0;
	diffintegral = 0;
	disterror_Stuck = distderivative_Stuck = prevdisterror_Stuck = 0;
	lastLatched = nPgmTime;
	lastLatched_Stuck = nPgmTime;
	hitWall = false;
	hitSomething = false;
	startingTime = nPgmTime + wallTime;
	goingBack = false;


	if(tight){
		driveErrorThreshold = Tight_driveErrorThreshold;
		turnErrorThreshold = Tight_turnErrorThreshold;
		} else {
		driveErrorThreshold = DGAF_driveErrorThreshold;
		turnErrorThreshold = DGAF_turnErrorThreshold;
	}

}

void stallDetection(){
	//Calculate if we can't move
	disterror_Stuck = getAvgEncoder(); //Get encoder value
	distderivative_Stuck = disterror_Stuck - prevdisterror_Stuck; //Calculate distance derivative
	prevdisterror_Stuck = disterror_Stuck; //Update previous distance error

	//When derivative error is less than certain value begin latching
	//If latched for more than certain time set hitWall to true
	if(abs(distderivative_Stuck) > 0.5){ //OLD 0.2
		lastLatched_Stuck = nPgmTime;
		hitSomething = false;
		} else {
		hitSomething =  nPgmTime - lastLatched_Stuck > stuckTimeout;
	}
}

//////////////////////////////////////////////////////////* DRIVE PID TASK *//////////////////////////////////////////////////////////////////
task PID_Drive(){
	while(true){



		//Runs the PID loop while isDriving is true and sets isDriving to false when done.
		while(isDriving){
			// Calculate both linear and difference errors
			disterror = linearDistance - getAvgEncoder(); //Calculate distance error
			differror = getGyro(); //Calculate difference error

			// Find the integral ONLY if within controllable range AND if the distance error is not equal to zero
			if(abs(distderivative) < 0.375 && disterror != 0){
				distintegral = distintegral + disterror;
				}else{
				distintegral = 0; //Otherwise, reset the integral
			}

			distderivative = disterror - prevdisterror; //Calculate distance derivative
			diffderivative = differror - prevdifferror; //Calculate difference derivative

			prevdisterror = disterror; //Update previous distance error
			prevdifferror = differror; //Update previous difference error

			distspeed = (disterror * distP) + (distintegral * distI) + (distderivative * distD); //Calculate distance speed
			diffspeed = (differror * diffP) + (diffintegral * diffI) + (diffderivative* diffD); //Calculate difference (turn) speed

			//Check that the speed is not exceeding the maximum set speed
			if(distspeed > maxspeed){
				distspeed = maxspeed;
			}

			//Check that the speed is not exceeding the maximum set speed
			if(distspeed < -maxspeed){
				distspeed = -maxspeed;
			}

			motor[FLD] = motor[BLD] = distspeed - diffspeed; //Set motor values
			motor[FRD] = motor[BRD] = distspeed + diffspeed; //Set motor values

			//Find sign of output
			if(distspeed > 0){
				direction = 1;
				} else {
				direction = -1;
			}

			//If close to range apply negative voltage to stop robot
			if(abs(disterror)<driveErrorThreshold){

				motor[FLD] = motor[BLD] = (15 * -direction);
			  motor[FRD] = motor[BRD] = (15 * -direction);
				wait1Msec(20); //OLD 100
				if(abs(distderivative) < 0.5){ //Once you stop moving reset
					isDriving = false;
					linearDistance = 0;
					motor[FLD] = motor[BLD] = 0;
			    motor[FRD] = motor[BRD] = 0;
				}
			}
			//stallDetection();
			hitSomething = false;
			if(!skills && hitSomething){
				//hover();
				linearDistance = 0;
				goingBack = true;
				isDriving = false;
			}


			wait1Msec(20);
		}

		//Runs the PID loop while isTurning is true and sets isTurning to false when done. Turns to the right by default for positive values.
		while(isTurning){

			disterror = turnAng - getGyro(); //Calculate distance error
			//differror = getLeftEncoder() - -1*getRightEncoder(); //Calculate difference error

			// Find the integral ONLY if within controllable range AND if the distance error is not equal to zero
			if( abs(disterror) < 6 && disterror != 0){
				distintegral = distintegral + disterror;
				}else{
				distintegral = 0; //Otherwise, reset the integral
			}

			distderivative = disterror - prevdisterror; //Calculate distance derivative
			diffderivative = differror - prevdifferror; //Calculate difference derivative

			prevdisterror = disterror; //Update previous distance error
			prevdifferror = differror; //Update previous difference error

			distspeed = (disterror * turnP) + (distintegral * turnI) + (distderivative * turnD); //Calculate distance speed
			//diffspeed = (differror * diffP) + (diffintegral * diffI) + (diffderivative* diffD); //Calculate difference (turn) speed
			diffspeed = 0; //TODO: Consider implementing this

			//Check that the speed is not exceeding the maximum set speed
			if(distspeed > maxspeed){
				distspeed = maxspeed;
			}

			//Check that the speed is not exceeding the maximum set speed
			if(distspeed < -maxspeed){
				distspeed = -maxspeed;
			}

			motor[FLD] = motor[BLD] = distspeed - diffspeed; //Set motor values
		  motor[FRD] = motor[BRD] = -1*(distspeed + diffspeed); //Set motor values //OLD *-1

					//Find direction of output
			if(distspeed > 0){
				direction = 1;
				} else {
				direction = -1;
			}

			//If close to range apply negative voltage to stop robot
			if(abs(disterror)<turnErrorThreshold){
				//autoTurn(20 * -direction); //OLD 15
				wait1Msec(5); //OLD 100
				if(abs(distderivative) < 0.5){ //Once you stop moving reset
					isTurning = false;
					turnAng = 0;
					motor[FLD] = motor[BLD] = 0;
			    motor[FRD] = motor[BRD] = 0;
				}
			}
			wait1Msec(20);
		}
		while(isWall){
			disterror = getAvgEncoder();
			differror = getGyro(); //Calculate difference error

			distderivative = disterror - prevdisterror; //Calculate distance derivative
			diffderivative = differror - prevdifferror; //Calculate difference derivative

			prevdisterror = disterror; //Update previous distance error
			prevdifferror = differror; //Update previous difference error

			diffspeed = (differror * diffP) + (diffintegral * diffI) + (diffderivative* diffD); //Calculate difference (turn) speed

			if(wallForward){
			motor[FLD] = motor[BLD] = wallPower - diffspeed; //Set motor values
			motor[FRD] = motor[BRD] = wallPower + diffspeed; //Set motor values
		} else {
			motor[FLD] = motor[BLD] = -wallPower - diffspeed; //Set motor values
			motor[FRD] = motor[BRD] = -wallPower + diffspeed; //Set motor values
		}

			//When derivative error is less than certain value begin latching
			//If latched for more than certain time set hitWall to true
			if(abs(distderivative) > 0.25){ //0.2
				lastLatched = nPgmTime;
				hitWall = false;
				} else {
				hitWall =  nPgmTime - lastLatched > 375; //OLD 250
			}

			//If hitWall and beginning time has passed stop motors and reset
			if(startingTime < nPgmTime && hitWall){
				motor[FLD] = motor[BLD] = 0; //Set motor values
				motor[FRD] = motor[BRD] = 0; //Set motor values
				isWall = false;
			}
			wait1Msec(20);
		}
		wait1Msec(20);
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Drives given distance in inches
void driveDistance(float dist, bool tolerance = false){
	initPID(tolerance);
	linearDistance = dist;
	isDriving = true;
	while(isDriving){
		wait1Msec(20);
	}
}
//Turns given angle in degrees (right = +)
void turnAngle(float ang, bool tolerance = false){
	initPID(tolerance);
	turnAng = ang;
	isTurning = true;
	while(isTurning){
		wait1Msec(20);
	}
}

void driveWall(bool forward){
	initPID(false);
	isWall=true;
	wallForward = forward;
	while(isWall){
		wait1Msec(20);
	}
}

