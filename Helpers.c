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
