void get_cone(int n_cones = 1){
	driveDistance(4);
	for (int i = 1; i <= n_cones; i++){
		driveDistance(12);
		wait_for_lift();
		set_lift_mode(lm_pickup);
		wait_for_lift();
		set_lift_mode(lm_score);
	}
	driveDistance((-12 * n_cones) -4);
}

void get_straight_mg(int offset = 0){
	mg_down();
	delay(500);
	driveDistance(49 - offset);
	mg_upHold();
	enable_intake();
	set_lift_mode(lm_score);
	delay(1500);
	//get_cone(2);
	driveDistance(-44);
}

void deliver_to_20(){
	driveWall(true);
	mg_downHold();
	driveDistance(-5);
	mg_upHold();
	driveDistance(-19);
}

void move_center(int dir){
	turnAngle(-45*dir);
	driveDistance(-30);
	turnAngle(-90*dir);
}

void deliver_to_10(){
	driveDistance(8);
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
	driveDistance(7);
	delay(500);
	turnAngle(95*dir);
	get_straight_mg(8);
	driveDistance(-5);
	move_center(-1*dir);
	deliver_to_10();
}

void bs_auto(bool isLeft){
	// 1 for left, -1 for right
	int dir = 2 * isLeft - 1;
	delay(300);
	get_straight_mg();
	move_center(dir);
	deliver_to_20();
	turnAngle(90*dir);
	driveDistance(-32);
	turnAngle(100*dir);
	driveWall(true);
}
