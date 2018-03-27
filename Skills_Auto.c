void corner_pass(bool is_long){
	set_odom(-137, 16, 30);
	driveDistance(-14);
	turnAngle(-108);
	driveWall(false, 60);

	set_odom(83, 11, 50);
	set_target(83, 13.5, 50.5);
	driveDistance(0);
	set_target(0, 13.5, 69);
	turnAngle(0);
	driveDistance(0);

	SensorValue[intake_piston] = 0;
	wait1Msec(400);
	CB_setpoint = CB_top_setpoint;
	target_p.p_x = 13.5;
	target_p.p_y = 90;

	driveToPoint(target_p);

	set_outer_goal(gs_down);
	target_p.p_y = 110;
	driveDistance(0);
	wait1Msec(300);
	SensorValue[intake_piston] = 1;
	turnAngle(0);
	set_inner_goal(gs_up);

	// Get second
	wait1Msec(400);
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
	SensorValue[intake_piston] = 0;
	wait1Msec(300);
	CB_setpoint = CB_top_setpoint;
	set_outer_goal(gs_up);
	//wait1Msec(1300);
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
	startDrive(60, 48);
	set_outer_goal(gs_down);
	startDrive(0, -5);
	grab();
	driveDistance(0);
	set_CB_target(CB_top_setpoint);
	CB_wait();
	drop();
	set_CB_target(CB_bottom_setpoint);
	CB_wait();
	driveDistance(8, 5);
	set_outer_goal(gs_up);
	driveDistance(0);
	grab();
	set_CB_target(CB_top_setpoint);

	point lineUp;
	lineUp.p_x = -6;
	lineUp.p_y = 116;
	lineUp.p_t = -5;
	CB_wait();
	drop();
}
