void back_out(){
	int oldmax = maxspeed;
	maxspeed = 60;
	driveDistance(-18);
	maxspeed = oldmax;
}

void corner_pass(bool is_long){
	set_odom(-137, 16, 30);
	//driveDistance(-14);
	back_out();
	driveDistance(3);
	turnAngle(-115);
	driveWall(false, 50);

	set_odom(83, 15, 50);
	set_target(83, 13.5, 50.5);
	driveDistance(0);
	set_target(0, 13.5, 69);
	turnAngle(0);
	driveDistance(0);

	grab();
	set_CB_target(CB_top_setpoint);
	target_p.p_x = 13.5;
	target_p.p_y = 90;

	driveToPoint(target_p);

	set_outer_goal(gs_down);
	driveToButton(80, dgtl10);
	pos_p.p_y = 96;
	target_p.p_y = 110;
	driveDistance(0);
	drop();
	turnAngle(0);
	set_inner_goal(gs_up);

	// Get second
	CB_setpoint = CB_hover_setpoint;
	driveDistance(-5);
	turnAngle(45);
	//startDrive(6, 3);
	startDrive(23, 3);
	CB_setpoint = CB_bottom_setpoint;
	//driveDistance(6);
	//turnAngle(20);
	set_inner_goal(gs_down);
	driveDistance(0);
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
	lineUp.p_x = 94;
	lineUp.p_y = 116;
	lineUp.p_t = 45;
	splineDest(lineUp, 24);

	driveWall(true);
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

void center_pass(){
	set_odom(0,0,0);
	drop();
	set_cb_target(CB_bottom_setpoint);

	startDrive(60, 23);
	set_outer_goal(gs_down);
	startDrive(0, -5);
	grab();
	driveDistance(0);
	set_CB_target(CB_top_setpoint);
	CB_wait();
	drop();
	set_CB_target(CB_bottom_setpoint);
	CB_wait();
	startDrive(8, 5);
	set_outer_goal(gs_up);
	driveDistance(0);
	grab();
	set_CB_target(CB_top_setpoint);

	point lineUp;
	lineUp.p_x = -4;
	lineUp.p_y = 116;
	lineUp.p_t = 0;
	splineDest(lineUp, 20);
	driveWall(true);
	CB_wait();
	set_inner_goal(gs_down);
	drop();
	set_odom(0,0,0);

	back_out();
	turnAngle(-90);
	driveDistance(22);
	turnAngle(-90);
	driveWall(false, 50);
}
