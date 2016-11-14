int liftState = 0;
    
//Left motor ticks increase if it is moving forward
//Right motor ticks increases if it is moving backwards

void setDriveSpeed(int frontLeft, int frontRight, int backLeft, int backRight) {
    motor[FRONT_RIGHT_DRIVE_MOTOR] = - frontRight;
    motor[BACK_RIGHT_DRIVE_MOTOR] = - backRight;
    motor[FRONT_LEFT_DRIVE_MOTOR] = frontLeft;
    motor[BACK_LEFT_DRIVE_MOTOR] = backLeft;
}

void setLiftSpeed(int speed) {
    motor[L_BOT_MOTOR] = -speed;
    motor[L_TOP_MOTOR] = -speed;
    motor[R_BOT_MOTOR] = speed;
    motor[R_TOP_MOTOR] = speed;
}

void setClawSpeed(int speed) {
    motor[LEFT_CLAW_MOTOR] = speed;
    motor[RIGHT_CLAW_MOTOR] = -speed;
}

//takes in desired state, returns actual state
int runLiftControlLoop(int state) {
    int liftPotValue = SensorValue[LIFT_POT];

   
    if(state == 0) {
            if (liftPotValue > LIFT_POTENTIOMETER_VALUE_LOW){
                setLiftSpeed(LIFT_DOWN_SPEED);
                return 1;
            }
            else if (liftPotValue <= LIFT_POTENTIOMETER_VALUE_LOW){
                setLiftSpeed(LIFT_DOWN_HOLDING_SPEED);
                return 0;
            }
        }
        else if (state==1) {
            if (liftPotValue < LIFT_POTENTIOMETER_VALUE_HIGH) {
                setLiftSpeed(LIFT_UP_SPEED);
                return 0;
            }
            else if (liftPotValue >= LIFT_POTENTIOMETER_VALUE_HIGH) {
                setLiftSpeed(LIFT_UP_HOLDING_SPEED);
                return 1;
            }
        }
        return 2;
}

void moveLift(int state) {
	while (runLiftControlLoop(state) != state) {
		//wait
		wait1Msec(10);
	}
}
