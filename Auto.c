void get_cone(int n_cones = 1){
	for (int i = 1; i <= n_cones; i++){
		driveDistance(12);
		wait_for_lift();
		set_lift_mode(lm_pickup);
		wait_for_lift();
		set_lift_mode(lm_score);
	}
	driveDistance(-12 * n_cones);
}

void get_straight_mg(int offset = 0){
	mg_down();
	delay(500);
	driveDistance(47 - offset);
	mg_upHold();
	enable_intake();
	set_lift_mode(lm_score);
	driveDistance(-42);
}

void deliver_to_20(){
	driveWall(true);
	mg_downHold();
	driveDistance(-5);
	mg_upHold();
	driveDistance(-15);
}

void move_center(int dir){
	turnAngle(-45*dir);
	driveDistance(-27);
	turnAngle(-90*dir);
}

void deliver_to_10(){
	driveDistance(4);
	mg_downHold();
	driveDistance(-7);
	mg_up();
}

void simple_auto(bool isLeft){
	// 1 for left, -1 for right
	int dir = 2 * isLeft - 1;
	delay(300);
	get_straight_mg();
	move_center(dir);
	deliver_to_20();
	turnAngle(90*dir);
	driveDistance(-32);
	turnAngle(45*dir);
	driveWall(false);
	driveDistance(8);
	delay(500);
	turnAngle(90*dir);
	get_straight_mg(8);
	move_center(-1*dir);
	deliver_to_10();
}
