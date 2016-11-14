#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    LIFT_POT,       sensorPotentiometer)
#pragma config(Sensor, I2C_1,  DRIVE_LEFT_MOTOR_ENCODER, sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  DRIVE_RIGHT_MOTOR_ENCODER, sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           DRIVE_FRONT_RIGHT_MOTOR, tmotorVex393HighSpeed_HBridge, openLoop)
#pragma config(Motor,  port2,           DRIVE_BACK_RIGHT_MOTOR, tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_2)
#pragma config(Motor,  port3,           LIFT_LEFT_BOT_MOTOR,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           LIFT_LEFT_TOP_MOTOR,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           LIFT_RIGHT_BOT_MOTOR,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           LIFT_RIGHT_TOP_MOTOR,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           CLAW_LEFT_MOTOR, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           CLAW_RIGHT_MOTOR, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           DRIVE_LEFT_BACK_MOTOR, tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port10,          DRIVE_LEFT_FRONT_MOTOR, tmotorVex393HighSpeed_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX2)
#include "Constants.c"
#include "PID.c"
#include "RobotFunctions.c"
#include "Opcontrol.c" //for whatever reason, this needs to be above the below pragma and includes
#include "Autonomous.c"

// Select Download method as "competition"
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"

//PRE-AUTONOMOUS

void pre_auton()
{
  // Set bStopTasksBetweenModes to false if you want to keep user created tasks
  // running between Autonomous and Driver controlled modes. You will need to
  // manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;
  SensorValue[DRIVE_LEFT_MOTOR_ENCODER] = 0;
  SensorValue[DRIVE_RIGHT_MOTOR_ENCODER] = 0;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

//AUTONOMOUS CONTROL TASK

task autonomous()
{
  runAutonomousLoop();
}

//USER CONTROL TASK

task usercontrol()
{
	runOpcontrolLoop();  
}
