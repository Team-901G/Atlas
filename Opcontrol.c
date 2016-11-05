//claw is not moving with two stars at motor power 50

void runOpcontrolLoop() {
	nMotorEncoder[LEFT_MOTOR_ENCODER] = 0;
	nMotorEncoder[RIGHT_MOTOR_ENCODER] = 0;
	int highSpeed = 127;
  int lowSpeed = 64;
  int clawOpen = 128;
  int clawClose = 70;
  while (true)
  {
  if (vexRT[Btn7U] == 1) {
		motor[L_BOT_MOTOR] = -highSpeed;
		motor[L_TOP_MOTOR] = -highSpeed;
		motor[R_BOT_MOTOR] = highSpeed;
		motor[R_TOP_MOTOR] = highSpeed;
	}
	else if (vexRT[Btn7D]==1) {
		motor[L_BOT_MOTOR] = highSpeed;
		motor[L_TOP_MOTOR] = highSpeed;
		motor[R_BOT_MOTOR] = -highSpeed;
		motor[R_TOP_MOTOR] = -highSpeed;
	}
	else if (vexRT[Btn7L] == 1) {
		motor[L_BOT_MOTOR] = -lowSpeed;
		motor[L_TOP_MOTOR] = -lowSpeed;
		motor[R_BOT_MOTOR] = lowSpeed;
		motor[R_TOP_MOTOR] = lowSpeed;
	}
	else if (vexRT[Btn7R]==1) {
		motor[L_BOT_MOTOR] = lowSpeed;
		motor[L_TOP_MOTOR] = lowSpeed;
		motor[R_BOT_MOTOR] = -lowSpeed;
		motor[R_TOP_MOTOR] = -lowSpeed;
	}
	else {
		motor[L_BOT_MOTOR] = 0;
		motor[L_TOP_MOTOR] = 0;
		motor[R_BOT_MOTOR] = 0;
		motor[R_TOP_MOTOR] = 0;
		}

	if(vexRT[Btn6U] == 1){
			motor[LEFT_CLAW_MOTOR] = clawOpen;
			motor[RIGHT_CLAW_MOTOR] = -clawOpen;
		}
	else if(vexRT[Btn6D] == 1){
			motor[LEFT_CLAW_MOTOR] = -clawClose;
			motor[RIGHT_CLAW_MOTOR] = clawClose;
		}
	else{
			motor[LEFT_CLAW_MOTOR] = 0;
			motor[RIGHT_CLAW_MOTOR] = 0;
		}

	//drive control
	int frontRightMotorSpeed = -((vexRT[Ch3] - vexRT[Ch4]) - vexRT[Ch1]);
	int backRightMotorSpeed = -((vexRT[Ch3] - vexRT[Ch4]) + vexRT[Ch1]);
 	int frontLeftMotorSpeed = ((vexRT[Ch3] + vexRT[Ch4]) + vexRT[Ch1]);
 	int backLeftMotorSpeed = ((vexRT[Ch3] + vexRT[Ch4]) - vexRT[Ch1]);


 	motor[FRONT_RIGHT_DRIVE_MOTOR] = frontRightMotorSpeed;
 	motor[BACK_RIGHT_DRIVE_MOTOR] = backRightMotorSpeed;
 	motor[FRONT_LEFT_DRIVE_MOTOR] = frontLeftMotorSpeed;
 	motor[BACK_LEFT_DRIVE_MOTOR] = backLeftMotorSpeed;
 	
	
  }
}
