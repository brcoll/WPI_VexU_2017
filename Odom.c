#ifndef ODOM_C
#define ODOM_C
double g_x = 0;
double g_y = 0;
double g_t = 0;

double odom_reset = false;

#define turnScale 0.239

float ticksToIn(long ticks){
	return ((float)ticks / 360.0) * 3.25 * PI;
}

task Odometry(){
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
		}

		lastLeft = leftCount;
		lastRight = rightCount;

		leftIn = ticksToIn(leftTicks);
		rightIn = ticksToIn(rightTicks);

		avgIn = (leftIn + rightIn)/2.0;

		theta += (leftTicks - rightTicks) * turnScale;

		if (theta > 180)theta -= 360;
		else if (theta < -180) theta += 360;

		g_y += avgIn * cos ((theta * 3.14) / 180);
		g_x += avgIn * sin ((theta * 3.14) / 180);
		g_t = theta;
		delay(20);
	}
}

//void autoZero(float coord_offset){

//}

#endif
