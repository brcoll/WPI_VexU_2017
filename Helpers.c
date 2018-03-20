
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

int removeDeadband(int input, int deadband = 20) {
	if (abs(input) > deadband) {
		return input;
	}
	else {
		return 0;
	}
}

void wait_user(){
	while (!vexRT(Btn7U)){
		wait1Msec(20);
	}
}
