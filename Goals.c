// Goals.c
//
// Oliver Jay
//
// Code to control skills mobile goals

enum goal_state{
	gs_down,
	gs_up,
	gs_unknown
};

int cycles_to_lift_inner = -0;

goal_state outer_goal_state = gs_unknown;
goal_state inner_goal_state = gs_unknown;


void set_inner_goal(goal_state new_state){
	if (new_state != inner_goal_state){
		inner_goal_state = new_state;
		SensorValue[inner_goal] = inner_goal_state;
	}
}

void set_outer_goal(goal_state new_state){
	if (new_state != outer_goal_state){
		if (new_state == gs_up){
			cycles_to_lift_inner = 20;
		}
		else {
			cycles_to_lift_inner = -1;
			set_inner_goal(new_state);
		}
		outer_goal_state = new_state;
		SensorValue[outer_goal] = !outer_goal_state;
	}
}

void update_goals(){
	if (cycles_to_lift_inner == 0){
		set_inner_goal(gs_up);
	}
	if (cycles_to_lift_inner >= 0){
		cycles_to_lift_inner --;
	}
}
