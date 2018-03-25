void corner_pass(bool is_long){
	set_odom(-137, 16, 30);
	driveDistance(-14);
	turnAngle(-108);
	driveWall(false, 60);

	set_odom(83, 9, 50);
	set_target(83, 12, 50.5);
	driveDistance(0);
	set_target(0, 12, 67);
	turnAngle(0);
	driveDistance(0);

	SensorValue[intake_piston] = 0;
	wait1Msec(400);
	CB_setpoint = CB_top_setpoint;
	target_p.p_x = 12;
	target_p.p_y = 90;
	driveDistance(0);
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
	turnAngle(45);
	startDrive(6, 3);
	//startDrive(21, 3);
	CB_setpoint = CB_bottom_setpoint;
	driveDistance(0);
	turnAngle(20);
	set_inner_goal(gs_down);
	driveDistance(18);
	SensorValue[intake_piston] = 0;
	wait1Msec(300);
	CB_setpoint = CB_top_setpoint;
	set_outer_goal(gs_up);
	wait1Msec(1300);
	SensorValue[intake_piston] = 1;
	target_p.p_t = 90;
	turnAngle(0);





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