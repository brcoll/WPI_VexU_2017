int CB_top_setpoint = 2800;
int CB_bottom_setpoint = 630;
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

void set_CB(int voltage){
	motor[CB1] = motor[CB2] = voltage;
}

task Control_CB(){
	while (true){
		CB_position = SensorValue[cb_pot];
		CB_error = CB_setpoint - CB_position;
		CB_power = ranged_output(CB_error * CB_Kp);
		set_CB(CB_power);
	}
}
