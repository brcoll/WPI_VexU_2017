int CB_top_setpoint = 2900;
int CB_bottom_setpoint = 610;
int CB_hover_setpoint = 1500;
int CB_setpoint = CB_bottom_setpoint;

enum CB_state{
	CBS_manual,
	CBS_idle,
	CBS_lifting,
	CBS_pausing,
	CBS_dropping,
	CBS_lowering,
	CBS_grabbing
};

float CB_Kp = 0.2;

float CB_error = 0;
int CB_position = 600;
float CB_power = 0;

int CB_good_loops = 0;

void set_cb_target(int new_setpoint){
	CB_setpoint = new_setpoint;
	CB_good_loops = 0;
}

void set_CB_pwr(int voltage){
	motor[CB1] = motor[CB2] = voltage;
}

void grab(){
	SensorValue[intake_piston] = 0;
	wait1Msec(300);
}

void drop(){
	SensorValue[intake_piston] = 1;
	wait1Msec(300);
}

task Control_CB(){
	while (true){
		CB_position = SensorValue[cb_pot];
		CB_error = CB_setpoint - CB_position;
		CB_power = ranged_output(CB_error * CB_Kp);
		set_CB_pwr(CB_power);
		wait1Msec(10);
		if (abs(CB_error) > 130) {
			CB_good_loops = 0;
		}
		else {
			CB_good_loops ++;
		}
	}
}

void cb_wait(){
	bool done = false;
	while (!disabled && !((CB_setpoint == CB_top_setpoint && cb_good_loops > 60) || (CB_setpoint == CB_bottom_setpoint && cb_good_loops > 10))){
		wait1Msec(10);
	}
}
