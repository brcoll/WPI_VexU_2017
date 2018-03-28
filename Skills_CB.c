int CB_top_setpoint = 2900;
int CB_bottom_setpoint = 610;
int CB_hover_setpoint = 1500;
int CB_threshold = 150;
int CB_hold_pwr = 20;
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
float CB_Kd = .01;

float CB_error = 0;
float CB_derivative = 0;
float CB_last = 0;
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
	if (SensorValue[intake_piston]){
		SensorValue[intake_piston] = 0;
		wait1Msec(300);
	}
}

void drop(){
	if (!SensorValue[intake_piston]){
		SensorValue[intake_piston] = 1;
		wait1Msec(300);
	}
}

task Control_CB(){
	while (true){
		CB_position = SensorValue[cb_pot];
		CB_error = CB_setpoint - CB_position;
		CB_derivative = CB_position - CB_last;
		CB_power = ranged_output(CB_error * CB_Kp - CB_derivative * CB_Kd);
		if ((CB_setpoint == CB_top_setpoint && CB_error > CB_threshold) ||
			(CB_setpoint == CB_bottom_setpoint && CB_error < -CB_threshold)) {
			CB_good_loops = 0;
		}
		else {
			CB_good_loops ++;
			CB_power = CB_power > 0 ? CB_hold_pwr : - CB_hold_pwr;
		}
		set_CB_pwr(CB_power);
		CB_last = CB_position;
		wait1Msec(10);
	}
}

void cb_wait(){
	bool done = false;
	while (!disabled && !((CB_setpoint == CB_top_setpoint && cb_good_loops > 60) || (CB_setpoint == CB_bottom_setpoint && cb_good_loops > 10))){
		wait1Msec(10);
	}
}
