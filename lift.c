int fb_pickup_setpoint = 630;
int fb_hold_setpoint = 1700;
int fb_score_setpoint = 2600;

float fb_p = 0.15;
int fb_tolerance = 200;
int fb_setpoint, fb_error, fb_output;
bool fb_done = false;

int lift_bottom_setpoint = 1200;
int lift_top_setpoint = 2200;
int lift_up_threshold = 1500;

float lift_p = 0.3;
int lift_tolerance = 20;
int lift_setpoint, lift_error, lift_output;
bool lift_done = false;

bool lift_enabled = true;

bool last_reset = false;

bool scoring = false;

void run_lift(int voltage){
	motor[RL] = motor[LL] = removeDeadband(voltage);
}

void run_fb(int voltage){
	motor[L4B] = motor[R4B] = voltage;
}

void set_lift(int new_setpoint){
	if (lift_setpoint != new_setpoint){
		lift_setpoint = new_setpoint;
		lift_done = false;
	}
}

void set_fb(int new_setpoint){
	if (fb_setpoint != new_setpoint){
		fb_setpoint = new_setpoint;
		fb_done = false;
	}
}

bool wait_for_lift(long timeout = 10000){
	clearTimer(T1);
	fb_done = false;
	lift_done = false;
	while(!fb_done || !lift_done){
		if (time1[T1] < timeout){
			lift_enabled = false;
			return false;
		}
		wait1Msec(20);
	}
	return true;
}

void raise_cone(bool hold = false){
	set_fb(fb_hold_setpoint);
	set_lift(lift_top_setpoint);
	if (hold){
		wait_for_lift();
	}
	last_reset = false;
}

void score_cone(bool hold = false){
	set_fb(fb_score_setpoint);
	if(fb_done){
		set_lift(lift_bottom_setpoint);
	}
}

void reset_lift(bool hold = false){
	if(!last_reset && SensorValue[lift_pot] > lift_up_threshold || scoring && !lift_done){
		score_cone();
		scoring = true;
	}
	else {
		scoring = false;
		fb_done = false;
		set_fb(fb_hold_setpoint);
		set_lift(lift_bottom_setpoint);
	}
	if (hold){
		wait_for_lift();
	}
	last_reset = true;
}

void pickup_cone(bool hold = false){
	fb_setpoint = fb_pickup_setpoint;
	lift_setpoint = lift_bottom_setpoint;
	if (hold){
		wait_for_lift();
	}
}

void clear_stack(){

}

task lift_intake_task(){
	lift_setpoint = lift_bottom_setpoint;
	while(true){
		fb_error = fb_setpoint - SensorValue(fb_pot);
		fb_output = (0.5*(fb_error * fb_P)) + (0.5*fb_output);

		fb_output = ranged_output(fb_output);

		if((fb_setpoint == fb_hold_setpoint || fb_setpoint == fb_score_setpoint) && abs(fb_error) < fb_tolerance){
			fb_output = 0;
		}

		fb_done = abs(fb_error) < fb_tolerance;

		if(!lift_enabled){
			fb_output = 0;
		}

		run_fb(fb_output);

		lift_error = lift_setpoint - SensorValue(lift_pot);
		lift_output = (0.5 * (lift_error * lift_p)) + (0.5 * lift_output);

		lift_output = ranged_output(lift_output);

		lift_done = abs(lift_error) < lift_tolerance;

		if(!lift_enabled){
			lift_output = false;
		}

		run_lift(lift_output);
		wait1Msec(20);
	}
}
