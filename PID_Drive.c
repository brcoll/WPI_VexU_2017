//Drive control vars
bool isDriving = false;
bool isTurning = false;
float linearDistance = 0;
float turnAng = 0;
int maxspeed = MAX_VOLTAGE;

#define ANG_CONST 123 //TODO: Add actual tuned values

int driveErrorThreshold = 5;
int turnErrorThreshold = 5;

//Linear variables
int disterror, differror, distintegral, diffintegral, distderivative, diffderivative, distspeed, diffspeed, prevdisterror, prevdifferror = 0;

//TODO: Tune these!!!!
//PLACEHOLDER VALUES LINEAR DRIVE
float distP = .5;
float distI = 0.025;
float distD = 0.2;
//PLACEHOLDER VALUES STEADY DRIVE
float diffP = 1;
float diffI = 0.05;
float diffD = 0.2;

//////////////////////////////////////////////////////////* DRIVE PID TASK *//////////////////////////////////////////////////////////////////
// This task relies on executing two loops, one for driving and one for turning. Could definitely be optimized and number of lines reduced, but
// we're on a time frame. TODO for later.
task PID_Drive(){
	while(true){

		//Executes once when isDriving or isTurning is flipped true.
		if(isDriving || isTurning){
			resetEncoders();
			prevdisterror = 0;
			prevdifferror = 0;
			distintegral = 0;
			diffintegral = 0;
		}

		//Runs the PID loop while isDriving is true and sets isDriving to false when done.
		while(isDriving){
			// Calculate both linear and difference errors
			disterror = linearDistance - ((SensorValue(leftEncoder) + SensorValue(rightEncoder))/2); //Calculate distance error
			differror = SensorValue(leftEncoder) - SensorValue(rightEncoder); //Calculate difference error

			// Find the integral ONLY if within controllable range AND if the distance error is not equal to zero
			if( abs(disterror) < 60 && disterror != 0){
				distintegral = distintegral + disterror;
				} else {
				distintegral = 0; //Otherwise, reset the integral
			}

			// Find the integral ONLY if within controllable range AND if the difference error is not equal to zero
			if(abs(differror) < 60 && differror != 0){
				diffintegral = diffintegral + differror;
				} else {
				diffintegral = 0; //Otherwise, reset the integral
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

			if(disterror<driveErrorThreshold){
				isDriving = false;
				linearDistance = 0;
			}
			wait1Msec(20);
		}

		//Runs the PID loop while isTurning is true and sets isTurning to false when done. Turns to the right by default for positive values.
		while(isTurning){

			disterror = turnAng*ANG_CONST - ((SensorValue(leftEncoder) + -1*SensorValue(rightEncoder))/2); //Calculate distance error
			differror = SensorValue(leftEncoder) - -1*SensorValue(rightEncoder); //Calculate difference error

			// Find the integral ONLY if within controllable range AND if the distance error is not equal to zero
			if( abs(disterror) < 60 && disterror != 0){
				distintegral = distintegral + disterror;
				} else {
				distintegral = 0; //Otherwise, reset the integral
			}

			// Find the integral ONLY if within controllable range AND if the difference error is not equal to zero
			if(abs(differror) < 60 && differror != 0){
				diffintegral = diffintegral + differror;
				} else {
				diffintegral = 0; //Otherwise, reset the integral
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
			motor[FRD] = motor[BRD] = -1*(distspeed + diffspeed); //Set motor values

			if(disterror<turnErrorThreshold){
				isTurning = false;
				turnAng = 0;
			}
			wait1Msec(20);
		}

		wait1Msec(20);
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveDist(float dist){
	linearDistance = dist;
	isDriving = true;
}

void turnAngle(float ang){
	turnAng = ang;
	isTurning = true;
}
