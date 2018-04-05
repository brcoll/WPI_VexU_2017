/*
Odomotry and coordinate library
Author: Oliver Jay
Adapted from point to point naviagaiton code from RBE2001
by Max Westwater, Sam White, and Oliver Jay
*/
#ifndef ODOM_C
#define ODOM_C

typedef struct _point{
	float p_x;
	float p_y;
	float p_t;
}point;

float g_x = 0;
float g_y = 0;
float g_t = 0;

float tOffset = 0;

bool odom_reset = false;

point target_p;
point pos_p;

#define SONAR_TO_IN 152.3
#define UPDATE_WEIGHT .06
#define WALL_READ_DIST 20

float distToWall(){
	return SensorValue[wall]/SONAR_TO_IN + 5.5
}

float wallDist = 0;

void senseWall(){
	if(pos_p.p_t % 90 < 3 && pos_p.p_t % 90 > -3 && distToWall() < WALL_READ_DIST){
		int ang = round(pos_p.p_t / 90.0);
		if (ang == 1 && pos_p.p_y > (144 - WALL_READ_DIST)){
			if (abs(distToWall() - (144 - pos_p.p_y)) < 5){
				pos_p.p_y = (pos_p.p_y * (1 - UPDATE_WEIGHT)) + ((144 - distToWall()) * UPDATE_WEIGHT);
			}
		}
		if (ang%4 == 0  && pos_p.p_x < WALL_READ_DIST){
			if (abs(distToWall() - pos_p.p_x) < 5){
				pos_p.p_x = (pos_p.p_x * (1 - UPDATE_WEIGHT)) + (distToWall() * UPDATE_WEIGHT);
			}
		}
	}
}

float ticksToIn(long ticks){
	return ((float)ticks / 1024 / 4) * 3.25 * PI;
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
		wallDist = distToWall();
		leftCount = SensorValue(leftEncoder);
		rightCount = SensorValue(rightEncoder);

		leftTicks = leftCount - lastLeft;
		rightTicks = rightCount - lastRight;

		if (odom_reset){
			odom_reset = false;
			leftTicks = 0;
			rightTicks = 0;
			theta = tOffset;
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
		senseWall();
		delay(5);
	}
}

void set_target(float t_t, float t_x, float t_y){
	target_p.p_t = t_t;
	target_p.p_x = t_x;
	target_p.p_y = t_y;
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

float get_coord_error(point p1, point p2, float theta = 1000){
	if (theta == 1000){
		theta = p1.p_t;
	}
	return proj_dist(p1, theta) - proj_dist(p2, theta);
}

void add_dist(point &position, float dist){
	position.p_x += dist * sin ((position.p_t * 3.14) / 180);
	position.p_y += dist * cos ((position.p_t * 3.14) / 180);
}

void copy_points(point &from, point &to){
	to.p_x = from.p_x;
	to.p_y = from.p_y;
	to.p_t = from.p_t;
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

void offset_on_line(point &p1, point reference, float offset){
	float current_offset = get_coord_error(p1, reference);
	add_dist(p1, offset - current_offset);
}

float angle_between_points(point start_p, point end_p){
  float pointX = end_p.p_x - start_p.p_x;
  float pointY = end_p.p_y - start_p.p_y;
  pointY = pointY ? pointY : pointY + .01;

	return ((atan(pointX/pointY) * 57.2958) + ((pointY < 0)* 180))%360 ;
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

void set_odom(float new_t, float new_x = 0, float new_y = 0){
	pos_p.p_x = new_x;
	pos_p.p_y = new_y;
	pos_p.p_t = new_t;
	target_p.p_x = new_x;
	target_p.p_y = new_y;
	target_p.p_t = new_t;
	tOffset = new_t;
	SensorValue(leftEncoder) = 0;
	SensorValue(rightEncoder) = 0;
	odom_reset = true;
}

#endif
