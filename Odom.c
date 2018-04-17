/*
Odomotry and coordinate library
Author: Oliver Jay
Adapted from point to point naviagaiton code from RBE2001
by Max Westwater, Sam White, and Oliver Jay
*/

#ifndef ODOM_C
#define ODOM_C

// Coordinate structure
typedef struct _point{
	float p_x;
	float p_y;
	float p_t;
}point;

// Initialization offset for theta
float tOffset = 0;

// Flag to reset odometry
bool odom_reset = false;

point target_p; // Target position
point pos_p; // Current position
bool is_left = true;

#define SONAR_TO_IN 148 // Conversion factor (reads raw for better resolution)
#define UPDATE_WEIGHT .06 // For exponential smoothing
#define WALL_READ_DIST 20 // Threshold distance to wall

#ifdef SKILLS
// Conversion factor
float distToWall(){
	return SensorValue[wall]/SONAR_TO_IN + 4.5;
}

// Current distance to wall (for debugging)
float wallDist = 0;

// Updates odometery off the wall
void senseWall(){
	if(pos_p.p_t % 90 < 8 && pos_p.p_t % 90 > -8 && distToWall() < WALL_READ_DIST){ // Valid update conditions
		int ang = round(pos_p.p_t / 90.0); // Get orientation
		if (ang == 1 && pos_p.p_y > (144 - WALL_READ_DIST)){ // Following far wall
			if (abs(distToWall() - (144 - pos_p.p_y)) < 5){ // Only update if close
				pos_p.p_y = (pos_p.p_y * (1 - UPDATE_WEIGHT)) + ((144 - distToWall()) * UPDATE_WEIGHT); // Exponential smoothing
			}
		}
		if (ang%4 == 0  && pos_p.p_x < WALL_READ_DIST){ // Following close wall
			if (abs(distToWall() - pos_p.p_x) < 5){ // Only update if close
				pos_p.p_x = (pos_p.p_x * (1 - UPDATE_WEIGHT)) + (distToWall() * UPDATE_WEIGHT); // Exponential smoothing
			}
		}
	}
}
#else
void senseWall(){
	return;
}
#endif

// Convert ticks to inches
float ticksToIn(long ticks){
	return ((float)ticks / TICKS_PER_REV / 4) * 3.25 * PI;
}

float getLeftEncoder() {
	return ticksToIn(SensorValue[leftEncoder]);
}

float getRightEncoder() {
	return ticksToIn(SensorValue[rightEncoder]);
}

// Main position tracking task
task Odometry(){
	// Initialize odom values
	target_p.p_x = 0;
	target_p.p_y = 0;
	target_p.p_t = 0;
	long lastLeft = SensorValue(leftEncoder);
	long lastRight = SensorValue(rightEncoder);
	long leftCount, rightCount, leftTicks, rightTicks;
	float leftIn, rightIn, avgIn, theta;

	while (true){
		// Read raw values
		wallDist = distToWall();
		leftCount = SensorValue(leftEncoder);
		rightCount = SensorValue(rightEncoder);

		// Get position change
		leftTicks = leftCount - lastLeft;
		rightTicks = rightCount - lastRight;

		// Reset values
		if (odom_reset){
			odom_reset = false;
			leftTicks = 0;
			rightTicks = 0;
			theta = tOffset;
		}

		// Stroe old values to measure change
		lastLeft = leftCount;
		lastRight = rightCount;

		// Read new values
		leftIn = ticksToIn(leftTicks);
		rightIn = ticksToIn(rightTicks);

		// Find position change
		avgIn = (leftIn + rightIn)/2.0;

		// Update angle
		theta += (leftTicks - rightTicks) * turnScale;

		// Wrap theta
		if (theta > 180)theta -= 360;
		else if (theta < -180) theta += 360;

		// Update coordinates based on motion and orientation
		pos_p.p_y += avgIn * cos ((theta * 3.14) / 180);
		pos_p.p_x += avgIn * sin ((theta * 3.14) / 180);
		pos_p.p_t = theta;

		// Update off of the wall
		senseWall();

		// Small loop delay
		delay(5);
	}
}

// Change the target setpoint
void set_target(float t_t, float t_x, float t_y){
	target_p.p_t = t_t;
	target_p.p_x = t_x;
	target_p.p_y = t_y;
}

// Returns the shortest angle between to headings
float angle_diff(float ang1, float ang2){
	float diff = ang1 - ang2;
	while (diff > 180) diff -= 360;
	while (diff < -180) diff += 360;
	return diff;
}

// Returns the component of a coordinate in a given heading
float proj_dist(point input, float theta){
	return input.p_x * sin ((theta * 3.14) / 180) + input.p_y * cos ((theta * 3.14) / 180);
}

// Finds the distance between to points in the direction of p1's heading
float get_coord_error(point p1, point p2, float theta = 1000){
	if (theta == 1000){
		theta = p1.p_t;
	}
	return proj_dist(p1, theta) - proj_dist(p2, theta);
}

// Moves a point in the direction of its heading by a given distance
void add_dist(point &position, float dist){
	position.p_x += dist * sin ((position.p_t * 3.14) / 180);
	position.p_y += dist * cos ((position.p_t * 3.14) / 180);
}

// Copy between point structs because RobotC is dumb
void copy_points(point &from, point &to){
	to.p_x = from.p_x;
	to.p_y = from.p_y;
	to.p_t = from.p_t;
}

// Returns the distance error toward the target
float get_drive_error(){
	return get_coord_error(target_p, pos_p);
}

// Returns the error in heading
float get_turn_error(){
	return angle_diff(target_p.p_t, pos_p.p_t);
}

// Moves the target setpoint forward a given amount in the direction of its heading
void advance_drive_target(float dist){
	add_dist(target_p, dist);
}

// Turn the target setpoint heading by a given angle
void advance_turn_target(float ang){
	target_p.p_t += ang;
}

// Moves a point to be offset a given distance from a given point along its heading
void offset_on_line(point &p1, point reference, float offset){
	float current_offset = get_coord_error(p1, reference);
	add_dist(p1, offset - current_offset);
}

// Returns the heading to face end_p from start_p
float angle_between_points(point start_p, point end_p){
  float pointX = end_p.p_x - start_p.p_x;
  float pointY = end_p.p_y - start_p.p_y;
  pointY = pointY ? pointY : pointY + .01;

	return ((atan(pointX/pointY) * 57.2958) + ((pointY < 0)* 180))%360 ;
}

// Translates from side centric to field centric
void translate_side(float *point_in, float heading, point &point_out){
	if (is_left){
		point_out.p_x = point_in[0];
		point_out.p_y = point_in[1];
		point_out.p_t = heading;
	} else {
		point_out.p_x = point_in[1];
		point_out.p_y = point_in[0];
		point_out.p_t = 90-heading;
	}
}

// Sets position, and target to zero
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

// Sets position and target to a given position
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
