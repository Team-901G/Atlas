//Left motor ticks increase if it is moving forward
//Right motor ticks increases if it is moving backwards

void moveForward(ticks) {
	while(nMotorEncoder[rightMotor] <= ticks){
		motor[FRONT_RIGHT_DRIVE_MOTOR] = frontRightMotorSpeed;
 		motor[BACK_RIGHT_DRIVE_MOTOR] = backRightMotorSpeed;
 		motor[FRONT_LEFT_DRIVE_MOTOR] = frontLeftMotorSpeed;
 		motor[BACK_LEFT_DRIVE_MOTOR] = highSpeed;
	}
}
void runAutonomousLoop() {
	//def 
}
