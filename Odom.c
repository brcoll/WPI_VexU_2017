#ifndef ODOM_C
#define ODOM_C


#define turnScale 0.239

typedef struct _point{
	float p_x;
	float p_y;
	float p_t;
}point;

float g_x = 0;
float g_y = 0;
float g_t = 0;

bool odom_reset = false;

point target_p;
point pos_p;

float ticksToIn(long ticks){
	return ((float)ticks / 360.0) * 3.25 * PI;
}

task Odometry(){
	target_p.p_x = 0;
	target_p.p_y = 0;
	target_p.p_t = 0;
	long lastLeft = SensorValue(leftEncoder);
	long lastRight = SensorValue(rightEncoder);
	long leftCount, rightCount, leftTicks, rightTicks;
	float leftIn, rightIn, avgIn, theta;
	while (true){
		leftCount = SensorValue(leftEncoder);
		rightCount = SensorValue(rightEncoder);

		leftTicks = leftCount - lastLeft;
		rightTicks = rightCount - lastRight;

		if (odom_reset){
			odom_reset = false;
			leftTicks = 0;
			rightTicks = 0;
			theta = 0;
		}

		lastLeft = leftCount;
		lastRight = rightCount;

		leftIn = ticksToIn(leftTicks);
		rightIn = ticksToIn(rightTicks);

		avgIn = (leftIn + rightIn)/2.0;

		theta += (leftTicks - rightTicks) * turnScale;

		if (theta > 180)theta -= 360;
		else if (theta < -180) theta += 360;

		pos_p.p_y += avgIn * cos ((theta * 3.14) / 180);
		pos_p.p_x += avgIn * sin ((theta * 3.14) / 180);
		pos_p.p_t = theta;
		delay(20);
	}
}

float angle_diff(float ang1, float ang2){
	float diff = ang1 - ang2;
	while (diff > 180) diff -= 360;
	while (diff < -180) diff += 360;
	return diff;
}

float proj_dist(point input, float theta){
	return input.p_x * sin ((theta * 3.14) / 180) + input.p_y * cos ((theta * 3.14) / 180);
}

float get_coord_error(point target, point position){
	return proj_dist(target, target.p_t) - proj_dist(position, target.p_t);
}

void add_dist(point &position, float dist){
	position.p_x += dist * sin ((position.p_t * 3.14) / 180);
	position.p_y += dist * cos ((position.p_t * 3.14) / 180);
}

float get_drive_error(){
	return get_coord_error(target_p, pos_p);
}

float get_turn_error(){
	return angle_diff(target_p.p_t, pos_p.p_t);
}

void advance_drive_target(float dist){
	add_dist(target_p, dist);
}

void advance_turn_target(float ang){
	target_p.p_t += ang;
}

void zero_odom(){
	pos_p.p_x = 0;
	pos_p.p_y = 0;
	pos_p.p_t = 0;
	target_p.p_x = 0;
	target_p.p_y = 0;
	target_p.p_t = 0;
	SensorValue(leftEncoder) = 0;
	SensorValue(rightEncoder) = 0;
	odom_reset = true;
}
//void autoZero(float coord_offset){

//}

#endif
