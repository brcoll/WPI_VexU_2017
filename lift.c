int fb_pickup_setpoint = 620;
int fb_hold_setpoint = 1700;
int fb_score_setpoint = 2600;

float fb_p = 0.15;
int fb_tolerance = 200;
int fb_setpoint, fb_error, fb_output;
bool fb_done = false;

int lift_bottom_setpoint = 1230;
int lift_top_setpoint = 2800;
int lift_up_threshold = 1290;
int lift_down_power = 20;

float lift_p = 0.4;
int lift_tolerance = 50;
int lift_setpoint, lift_error, lift_output;
bool lift_done = false;

bool intake_state = false;

bool clear_for_goal = false;

bool lift_enabled = true;

bool last_reset = false;

bool scoring = false;

bool is_clear = false;

float in_to_pot = 62;

int beam_offset = 200;

bool shouldOpen = false;


enum lift_mode
{
	lm_home,
	lm_pickup,
	lm_raise,
	lm_stationary,
	lm_score,
	lm_hold,
	lm_clear
};

enum lift_state
{
	ls_homing,
	ls_raising,
	ls_swinging,
	ls_holding,
	ls_picking_up,
	ls_clearing
};

lift_mode active_lift_mode;
lift_state active_lift_state;

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

void set_intake(bool open){
	if(intake_isAuto){
		open = false;
	}
	if (open != intake_state){
		SensorValue[intake] = open;
		intake_state = open;
	}
}

void set_lift_state(lift_state new_state){
	if(new_state != active_lift_state){
		active_lift_state = new_state;
	}
}

void set_lift_mode(lift_mode new_mode){
	if(new_mode != active_lift_mode && !clear_for_goal){
		active_lift_mode = new_mode;
		switch(new_mode){
			case lm_home:
			if(active_lift_state != ls_swinging && active_lift_state != ls_clearing && active_lift_state != ls_holding){
				set_lift_state(ls_homing);
			}
			break;

			case lm_pickup:
			set_lift(lift_bottom_setpoint);
			if(lift_done){
				set_lift_state(ls_picking_up);
			}
			else{
				set_lift_state(ls_homing);
			}
			break;

			case lm_raise:
			set_lift_state(ls_raising);
			break;

			case lm_stationary:
			set_fb(fb_hold_setpoint);
			set_lift_state(ls_raising);
			break;

			case lm_score:
			intake_isAuto = false;
			set_lift_state(ls_raising);
			break;

			case lm_hold:
			set_lift_state(ls_raising);
			break;

			case lm_clear:
			set_lift_state(ls_raising);
			break;
		}
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

bool goal_moving(bool is_moving){
	if(is_moving != clear_for_goal){
		if (is_moving){
			set_lift_mode(lm_clear);
		}
		clear_for_goal = is_moving;
	}
	return active_lift_state == ls_clearing;
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
	fb_setpoint = fb_hold_setpoint;
	lift_setpoint = lift_bottom_setpoint;
	while(true){
		switch(active_lift_state){

			case ls_homing:
			set_lift(lift_bottom_setpoint);
			if (lift_done){
				if(shouldOpen){
					set_intake(true);
					shouldOpen = false;
				}
				set_fb(fb_hold_setpoint);
				if(fb_done && active_lift_mode == lm_pickup){
					set_lift_state(ls_picking_up);
				}
			}
			break;

			case ls_raising:
			set_lift(lift_top_setpoint);
			if((SensorValue[beam] || lift_done) && active_lift_mode != lm_stationary && SensorValue[lift_pot] > lift_up_threshold){
				set_lift(SensorValue[lift_pot] + beam_offset);
				set_lift_state(ls_swinging);
			}
			break;

			case ls_swinging:
			set_fb(fb_score_setpoint);
			if(fb_done){
				if(active_lift_mode == lm_hold){
					set_lift_state(ls_holding);
					set_lift(SensorValue[lift_pot] - 4 * in_to_pot);
				}
				else if(active_lift_mode == lm_clear){
					set_lift_state(ls_clearing);
					set_lift(SensorValue[lift_pot] + 4 * in_to_pot);
					set_intake(true);
				}
				else{
					set_lift_state(ls_homing);
					shouldOpen = true;
					//set_intake(true);
				}
			}
			break;

			case ls_holding:
			set_intake(false);
			break;

			case ls_picking_up:
			set_fb(fb_pickup_setpoint);
			if(fb_done){
				set_intake(false);
			}
			break;

			case ls_clearing:
			set_intake(true);
			break;
		}


		fb_error = fb_setpoint - SensorValue(fb_pot);
		fb_output = (0.5*(fb_error * fb_P)) + (0.5*fb_output);

		fb_output = ranged_output(fb_output);

		if((fb_setpoint == fb_hold_setpoint || fb_setpoint == fb_score_setpoint) && abs(fb_error) < fb_tolerance){
			fb_output = 0;
		}
		else if(active_lift_state == ls_picking_up){
			fb_output = -127;
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

		if (lift_setpoint == lift_bottom_setpoint && SensorValue[lift_pot] < lift_tolerance + lift_setpoint){
			lift_output = lift_down_power;
			lift_done = true;
		}

		if(!lift_enabled){
			lift_output = 0;
		}

		run_lift(lift_output);
		wait1Msec(20);
	}
}
