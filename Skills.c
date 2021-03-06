#pragma config(Sensor, in1,    cb_pot,         sensorPotentiometer)
#pragma config(Sensor, dgtl1,  inner_goal,     sensorDigitalOut)
#pragma config(Sensor, dgtl2,  outer_goal,     sensorDigitalOut)
#pragma config(Sensor, dgtl3,  intake_piston,  sensorDigitalOut)
#pragma config(Sensor, dgtl4,  leftEncoder,    sensorQuadEncoder)
#pragma config(Sensor, dgtl6,  rightEncoder,   sensorQuadEncoder)
#pragma config(Sensor, dgtl8,  wall,           sensorSONAR_raw)
#pragma config(Motor,  port1,           LRD,           tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           LFR,           tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           LFD,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           CB1,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           LRR,           tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           RRR,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           CB2,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           RFD,           tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           RFR,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          RRD,           tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

#define MAX_VOLTAGE 127

#define turnScale .01152
// Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

#include "Skills_Constants.c"
#include "Helpers.c"
#include "Goals.c"
#include "Odom.c"
#include "Skills_Drive.c"
#include "PID_DriveV2.c"
#include "Skills_CB.c"
#include "Skills_Auto.c"
#include "Canceler.c"

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton()
{
  // Set bStopTasksBetweenModes to false if you want to keep user created tasks
  // running between Autonomous and Driver controlled modes. You will need to
  // manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task autonomous()
{
	startTask(Odometry);
  // ..........................................................................
  // Insert user code here.
  // ..........................................................................

  // Remove this function call once you have "real" code.
  AutonomousCodePlaceholderForTesting();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

int pot_val = 0;
float worst_bat = 0;
task usercontrol()
{
  // User control code here, inside the loop
	startTask(Control_CB);
	startTask(Odometry);
	startTask(PID_Drive);
	startTask(canceler);
  while (true)
  {
  	if (true || nImmediateBatteryLevel < worst_bat){
  		worst_bat = nImmediateBatteryLevel;
  	}

		if(vexRT(Btn7L)){
			//corner_pass(false);
			//driveDistance(24);
			turnAngle(0);
			//point first_goal;
			//first_goal.p_x = 12;
			//first_goal.p_y = 100;
			//first_goal.p_t = 0;
			//splineDest(first_goal);
		} else if (vexRT(Btn7R) && abs(vexRT(Ch1)) < 25){
			//turnAngle(90);
			//driveDistance(0);
			//delay(1000);
			//turnAngle(0);
			//point dest;
			//splineDest(dest, 20);
			programming_skills();
		} else if (vexRT(Btn7D) && abs(vexRT(Ch1)) < 25){
			////turnAngle(90);
			//set_odom(83, 7, 50);
			//set_odom(ACP_CP_T, ACP_CP_X, 50);
			set_odom(0, 0, 0);
			worst_bat = nImmediateBatteryLevel;
		} else if (vexRT(Btn7U) && abs(vexRT(Ch1)) < 25){
			//corner_pass(false);
			//center_pass(true);
			//back_out();
			//score_cone();
			driveDistance(0);
		} else {
			arcadeDrive();
			isDriving = false;
			isTurning = false;
		}
  	pot_val = SensorValue[cb_pot];
  	arcadeDrive();

  	if (vexRT(Btn5U)){
  		set_outer_goal(gs_up);
  	}
  	else if (vexRT(Btn5D)){
  		set_outer_goal(gs_down);
  	}

  	if(vexRT(Btn6U)){
  		set_inner_goal(gs_up);
  	}
  	else if (vexRT(Btn6D)){
  		set_inner_goal(gs_down);
  	}

  	if(vexRT(Btn8U)){
  		CB_setpoint = CB_top_setpoint;
  	}
  	else if (vexRT(Btn8D)){
  		CB_setpoint = CB_bottom_setpoint;
  	}

  	if(vexRT(Btn8R)){
  		SensorValue[intake_piston] = 0;
  	}
  	else if(vexRT(Btn8L)){
  		SensorValue[intake_piston] = 1;
  	}
  }
}
