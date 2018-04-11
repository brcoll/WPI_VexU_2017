void back_out(){
	int oldmax = maxspeed;
	maxspeed = 50;
	driveDistance(-19);
	maxspeed = oldmax;
}

#define ACP_CP_T 83
#define ACP_CP_X 10

void corner_pass(bool is_long){
	set_odom(-137, 16, 30);
	//driveDistance(-14);
	back_out();
	driveDistance(3);
	//set_CB_target(CB_bottom_setpoint);
	turnAngle(-115);
	driveWall(false, 50);

	set_odom(ACP_CP_T, ACP_CP_X, 50);
	set_target(ACP_CP_T, 12, 50.5);
	driveDistance(0, true);
	set_target(0, 13.5, 69);
	turnAngle(0, true);
	driveDistance(0);

	grab();
	set_CB_target(CB_top_setpoint);
	target_p.p_x = 13.5;
	target_p.p_y = 90;

	driveToPoint(target_p);

	set_outer_goal(gs_down);
	target_p.p_y = 114;
	driveDistance(0);
	set_inner_goal(gs_up);
	drop(true);
	turnAngle(0);
	wait_user(); // Temporary wait

	// Get second
	CB_setpoint = CB_hover_setpoint;
	driveDistance(-7, true);

	wait_user(); // Temporary wait
	turnAngle(15, true);
	wait_user(); // Temporary wait
	driveDistance(12);
	wait_user(); // Temporary wait
	turnAngle(46);
	wait_user(); // Temporary wait
	//startDrive(6, 3);
	startDrive(13, 3);
	CB_setpoint = CB_bottom_setpoint;
	//driveDistance(6);
	//turnAngle(20);
	set_inner_goal(gs_down);
	driveDistance(0);
	wait_user(); // Temporary wait
	grab();
	CB_setpoint = CB_top_setpoint;
	set_outer_goal(gs_up);
	target_p.p_t = 80;
	turnAngle(0);

	// Go place
	startDrive(40, 10);
	//SensorValue[intake_piston] = 1;
	//driveDistance(0);
	//turnAngle(45);
	//driveDistance(20);
	//turnAngle(-90);
	//driveWall(true);

	point lineUp;
	lineUp.p_x = 100;
	lineUp.p_y = 107;
	lineUp.p_t = 45;
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
		driveDistance(24, true);
		turnAngle(-90);
		driveWall(false, 50);
		set_cb_target(CB_bottom_setpoint);
		set_odom(-2,0,0);
		startDrive(60, 23);
		set_outer_goal(gs_down);
	}
	else {
		set_cb_target(CB_top_setpoint);
		startDrive(52, 23);
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
	set_CB_target(CB_top_setpoint);
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
	lineUp.p_x = -3;
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
