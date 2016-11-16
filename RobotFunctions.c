int liftState = 0;

//Left motor ticks increase if it is moving forward
//Right motor ticks increases if it is moving backwards
void initializeSensors() {
	SensorValue[DRIVE_LEFT_MOTOR_ENCODER] = 0;
  SensorValue[DRIVE_RIGHT_MOTOR_ENCODER] = 0;
  SensorValue[DRIVE_LEFT_BACK_QUAD] = 0;
  SensorValue[DRIVE_RIGHT_BACK_QUAD] = 0;
  SensorValue[DRIVE_LEFT_FRONT_QUAD] = 0;
  SensorValue[DRIVE_RIGHT_FRONT_QUAD] = 0;
  SensorValue[DRIVE_GYRO] = 0;
  wait1Msec(1000);
}


void setDriveSpeed(int frontLeft, int frontRight, int backLeft, int backRight) {
    motor[DRIVE_RIGHT_FRONT_MOTOR] = - frontRight;
    motor[DRIVE_RIGHT_BACK_MOTOR] = - backRight;
    motor[DRIVE_LEFT_FRONT_MOTOR] = frontLeft;
    motor[DRIVE_LEFT_BACK_MOTOR] = backLeft;
}

void setLiftSpeed(int speed) {
    motor[LIFT_LEFT_BOT_MOTOR] = -speed;
    motor[LIFT_LEFT_TOP_MOTOR] = -speed;
    motor[LIFT_RIGHT_BOT_MOTOR] = speed;
    motor[LIFT_RIGHT_TOP_MOTOR] = speed;
}

void setClawSpeed(int speed) {
    motor[CLAW_LEFT_MOTOR] = speed;
    motor[CLAW_RIGHT_MOTOR] = -speed;
}

//takes in desired state, returns actual state
int runLiftControlLoop(int state) {
    int liftPotValue = SensorValue[LIFT_POTENTIOMETER];


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
