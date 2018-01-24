void get_straight_mg(){
	mg_down();
	delay(500);
	driveDistance(47);
	mg_upHold();
	driveDistance(-42);
}

void deliver_to_20(){
	driveWall(true);
	mg_downHold();
	driveDistance(-4);
	mg_upHold();
	driveDistance(-13);
}

void simple_auto(bool isLeft){
	// 1 for left, -1 for right
	int dir = 2 * isLeft - 1;
	get_straight_mg();
	turnAngle(-45*dir);
	driveDistance(-22);
	turnAngle(-90*dir);
	deliver_to_20();
	turnAngle(-90*dir);
	driveDistance(22);
	turnAngle(-45*dir);
}
