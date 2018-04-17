void back_out(){
	int oldmax = max_speed;
	max_speed = 50;
	driveDistance(-19);
	max_speed = oldmax;
}

#define ACP_CP_T 83
#define ACP_CP_X 9

float slide_t = 0;

void corner_pass(bool is_long){
	set_odom(-137, 16, 30);

	back_out();
	driveDistance(3);
	set_CB_target(CB_bottom_setpoint);
	turnAngle(-115);
	driveWall(false, 70);

	slide_t = pos_p.p_t;

	if (pos_p.p_t < 87){ // Correct for if you miss line up
		set_odom(ACP_CP_T, ACP_CP_X, 50);
	}
	else {
		set_odom(90, ACP_CP_X, 52);
	}

	set_target(ACP_CP_T, 12, 50.5);
	driveDistance(0, true);
	set_target(0, 13.5, 69);
	turnAngle(0, true);
	driveDistance(0);

	grab();
	set_CB_target(CB_top_setpoint);
	target_p.p_x = 13.5;
	target_p.p_y = 90;
	target_p.p_t = 0;

	driveToPoint(target_p);

	set_outer_goal(gs_down);
	target_p.p_y = 110;
	driveDistance(0);
	set_inner_goal(gs_up);
	drop(true);
	target_p.p_t = 0;
	//turnAngle(0);
	wait_user(); // Temporary wait

	// Get second
	CB_setpoint = CB_hover_setpoint;
	//driveDistance(-7, true);

	wait_user(); // Temporary wait
	turnAngle(20, true);
	wait_user(); // Temporary wait
	set_cb_target(CB_bottom_setpoint);
	startDrive(8, 5);
	grab();
	CB_setpoint = CB_top_setpoint;
	driveDistance(0);
	wait_user(); // Temporary wait
	driveDistance(-4, true);
	wait_user(); // Temporary wait
	turnAngle(42);
	cb_wait();
	drop();
	wait_user(); // Temporary wait
	//startDrive(6, 3);
	startDrive(20, 4);
	//driveDistance(6);
	//turnAngle(20);
	set_inner_goal(gs_down);
	driveDistance(0);
	wait_user(); // Temporary wait
	grab();
	CB_setpoint = CB_top_setpoint;
	set_outer_goal(gs_up);
	target_p.p_t = 85;
	turnAngle(0);

	// Go place
	startDrive(50, 10);
	//SensorValue[intake_piston] = 1;
	//driveDistance(0);
	//turnAngle(45);
	//driveDistance(20);
	//turnAngle(-90);
	//driveWall(true);

	point lineUp;
	lineUp.p_x = 102;
	lineUp.p_y = 105;
	lineUp.p_t = 46;
	splineDest(lineUp, 24);

	driveWall(true, 127);
	set_inner_goal(gs_down);
	drop();

	//driveDistance(25);
 	// SensorValue[intake_piston] = 0;
	//delay(1000);
 	// CB_setpoint = CB_top_setpoint;
	//turnAngle(35);
	//driveDistance(30);
 	// SensorValue[intake_piston] = 1;

	//delay(2000);
	//driveDistance(0);
	//delay(2000);
	//turnAngle(-90);
	//delay(2000);
	//driveDistance(-27);
	//delay(2000);
	//turnAngle(-135);
}

void center_pass(bool first){
	set_odom(0,0,0);

	if (!first){
		drop();
		set_inner_goal(gs_down);
		back_out();
		turnAngle(-90);
		driveDistance(23, true);
		turnAngle(-90);
		driveWall(false, 50);
		set_cb_target(CB_bottom_setpoint);
		set_odom(-2,0,0);
		startDrive(60, 23);
		set_outer_goal(gs_down);
	}
	else {
		set_cb_target(CB_top_setpoint);
		startDrive(45, 23);
		set_outer_goal(gs_down);
		driveDistance(0);
		drop();

		set_cb_target(CB_bottom_setpoint);
		cb_wait();
	}
	target_p.p_y = 60;
	startDrive(0, -5);
	grab();
	driveDistance(0);
	pos_p.p_x = 0;
	set_CB_target(CB_top_setpoint);
	turnAngle(0);
	CB_wait();
	drop();
	set_CB_target(CB_bottom_setpoint);
	CB_wait();
	startDrive(9, 6);
	set_outer_goal(gs_up);
	driveDistance(0);
	grab();
	set_CB_target(CB_top_setpoint);

	point lineUp;
	lineUp.p_x = -5;
	lineUp.p_y = 116;
	lineUp.p_t = 0;
	splineDest(lineUp, 26);
	driveWall(true, 127);
	CB_wait();
	set_inner_goal(gs_down);
	drop();
	set_odom(0,0,0);
}

void programming_skills(){
	center_pass(true);
	center_pass(false);
	corner_pass(false);
	corner_pass(true);
	back_out();
}

void score_cone(){
	grab();
	set_cb_target(CB_top_setpoint);
	cb_wait();
	drop();
	set_cb_target(CB_bottom_setpoint);
}
