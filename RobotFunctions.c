int LIFT_HIGH_SPEED = 127;
int LIFT_LOW_SPEED = 64;
int CLAW_OPEN_SPEED = 128;
int CLAW_CLOSE_SPEED = -70;
int liftState = 0;
    
    //Simple Lifting Variables
int LIFT_UP_SPEED = 128;
int LIFT_DOWN_SPEED = -60;
//OLD POT VALUES:
    // max is 3686
    // min is 1655
    //int LIFT_POTENTIOMETER_VALUE_HIGH = 3500;
    //int LIFT_POTENTIOMETER_VALUE_LOW = 1800;
//NEW POT VALUES:
//min should be ~312
//min should be ~2033
int LIFT_POTENTIOMETER_VALUE_HIGH= 1853;
int LIFT_POTENTIOMETER_VALUE_LOW = 462;


int LIFT_DOWN_HOLDING_SPEED = 0;
int LIFT_UP_HOLDING_SPEED = 50;
    
    
    //claw is not moving with two stars at motor power 50
    
//Left motor ticks increase if it is moving forward
//Right motor ticks increases if it is moving backwards
void setDriveSpeed(int leftSide,int rightSide) {
  	motor[FRONT_RIGHT_DRIVE_MOTOR] = -rightSide;
 		motor[BACK_RIGHT_DRIVE_MOTOR] = -rightSide;
 		motor[FRONT_LEFT_DRIVE_MOTOR] = leftSide;
 		motor[BACK_LEFT_DRIVE_MOTOR] = leftSide;
}

void moveForward(int ticks, int speed) {
	while(SensorValue[LEFT_MOTOR_ENCODER] <= ticks){
	  setDriveSpeed(speed,speed);
	  writeDebugStreamLine("in loop, motor enc val %d",nMotorEncoder[LEFT_MOTOR_ENCODER]);
	}
	setDriveSpeed(0,0);
	nMotorEncoder[LEFT_MOTOR_ENCODER]=0;
	nMotorEncoder[RIGHT_MOTOR_ENCODER]=0;
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
    