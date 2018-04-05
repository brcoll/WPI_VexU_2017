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

goal_state outer_goal_state = gs_unknown;
goal_state inner_goal_state = gs_unknown;


void set_inner_goal(goal_state new_state){
	if (new_state != inner_goal_state){
		inner_goal_state = new_state;
		SensorValue[inner_goal] = inner_goal_state;
	}
}

void set_outer_goal(goal_state new_state){
	set_inner_goal(new_state);
	if (new_state != outer_goal_state){
		outer_goal_state = new_state;
		SensorValue[outer_goal] = !outer_goal_state;
	}
}
