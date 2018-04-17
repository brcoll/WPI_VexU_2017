// Returns the input, or bounded value if out of range
int ranged_output(int input, int max = 127){
	if (input > max){
		return max;
	}
	else if (input < -max){
		return -max;
	}
	else {
		return input;
	}
}

// same as above but a better name
int clamp(int input, int clampValue = 127){
	//Check that the speed is not exceeding the maximum set speed
	if(input > clampValue){
		input = clampValue;
	}

	//Check that the speed is not exceeding the maximum set speed
	if(input < -clampValue){
		input = -clampValue;
	}
	return input;
}

int max(int in1, int in2){
	if(in1 > in2){
		return in1;
		} else {
		return in2;
	}
}

// Removes deadband
int removeDeadband(int input, int deadband = 20) {
	if (abs(input) > deadband) {
		return input;
	}
	else {
		return 0;
	}
}

// Waits for user to continue execution
void wait_user(){
	while (!(vexRT(Btn7U)||vexRT(Btn7D))){
		wait1Msec(20);
	}
}
