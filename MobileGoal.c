// Mobile Goal Intake

int mg_topSetpoint = 2600;
int mg_bottomSetpoint = 680;

float mg_P = 1;

int mg_setpoint, mg_error, mg_output;
bool mg_done = false;

void mobileGoal(int voltage){
	motor(LMG) = motor(RMG) = voltage;
}

task mg_intake()
{
	mg_setpoint = mg_topSetpoint;
	while(true){


		mg_error = mg_setpoint - SensorValue(mg_pot);
		mg_output = mg_error * mg_P;


		//Check that the speed is not exceeding the maximum set speed
		if(mg_output > maxspeed){
			mg_output = maxspeed;
		}
		//Check that the speed is not exceeding the maximum set speed
		if(mg_output < -maxspeed){
			mg_output = -maxspeed;
		}


		if((SensorValue[upperLimit]) && mg_setpoint == mg_topSetpoint){
			mg_output = 0;
			mg_done = true;
		} else if((SensorValue[lowerLimit] && !SensorValue[upperLimit]) && mg_setpoint == mg_bottomSetpoint){
			mg_output = 0;
			mg_done = true;
		} else {
		  mg_done=false;
  	}

		mobileGoal(mg_output);
		wait1Msec(20);


	}
}

void mg_up(){
	mg_setpoint = mg_topSetpoint;
}

void mg_down(){
	mg_setpoint = mg_bottomSetpoint;
}

void mg_upHold(){
	mg_done = false;
	while(!mg_done){
		mg_setpoint = mg_topSetpoint;
	}
}

void mg_downHold(){
	mg_done = false;
	while(!mg_done){
	mg_setpoint = mg_bottomSetpoint;
	}
}
