//Convention: a positive turn angle means turning right
//a positive remaining distance implies a positive speed needs to be applied to close the gap
//this is assuming right rotation is positive on the gyro
//also assuming that claw opens with positive value to setClawSpeed, and pot gets bigger with claw more open

//TODO: check rotation on gyro, put in claw pot values
//TODO: see stuff above
//TODO: work out 1. drive differential PID values 2. claw PID values 3. lift PID values
	//if these don't show promise, switch to the other control type
//TODO: make routine work

typedef struct {
    float forwardTicks;     //in quad encoder ticks delta
    float strafeTicks;      //in quad encoder ticks delta
    float rotationAngle;    //in gyro angle delta
    float liftPosition;     //from 0 to 100, 0 is down
    float clawPosition;     //from 0 to 100, 0 is closed

} waypoint;


waypoint waypointsList[10];
int NUM_WAYPOINTS = 10;

PIDObject differentialDrivePID;
PIDObject rotationPID;
PIDObject liftPID;
PIDObject clawPID;

//---UTILITY FUNCTIONS---//
int sign(float f) {
	if (f >= 0) {return 1;}
	else {return -1;}

}

float map(float val,float min1,float max1, float min2, float max2) {
	return (((val-min1)/(max1-min1))*(max2-min2))+min2;
}

//---PID FUNCTIONS---//

float linearSpeedCurve(float remaining, float threshold) {
	if (fabs(remaining) > threshold) {
		return AUTON_DRIVE_MAX_SPEED;
		}
	else {
		return (remaining / threshold)* AUTON_DRIVE_MAX_SPEED;
	}
}

void initializeDrivePID(float kp, float ki, float kd) {
    intializePID(&differentialDrivePID,kp,ki,kd);
}

//should work for forwards, backwards, left turn, and right turn movement
//this only control driving straight -- right now the linearSpeedCurve acts as a P(no I no D) controller
//might want to change linearSpeedCurve into a PID controller
void updateDrivePID(int leftSpeed, int rightSpeed) {
    float dT = AUTON_LOOP_DELAY;

    float frontLeftTicksRemaining = - -SensorValue[DRIVE_LEFT_FRONT_QUAD];
    float backLeftTicksRemaining = - SensorValue[DRIVE_LEFT_BACK_QUAD];
    float leftAvgTicksRemaining = (frontLeftTicksRemaining + backLeftTicksRemaining)/2;

    float frontRightTicksRemaining =  - SensorValue[DRIVE_RIGHT_FRONT_QUAD];
    float backRightTicksRemaining = - -SensorValue[DRIVE_RIGHT_BACK_QUAD];
    float rightAvgTicksRemaining = (frontRightTicksRemaining + backRightTicksRemaining)/2;

    float tickDiff = fabs(rightAvgTicksRemaining) - fabs(leftAvgTicksRemaining);
    computePID(&differentialDrivePID,tickDiff,dT);

    //left side should compensate if right side is going faster (kP is positive)
    int leftOutputSpeed = sign(leftAvgTicksRemaining)*((int)(differentialDrivePID.output))+leftSpeed;
    int rightOutputSpeed = rightSpeed;

    setDriveSpeed(leftOutputSpeed,rightOutputSpeed,leftOutputSpeed,rightOutputSpeed);
}

//void initializeDrivePID() {
//   intializePID(rotationPID,0,0,0);
//}

void initializeLiftPID(float kp, float ki, float kd) {
    intializePID(&liftPID,kp,ki,kd);
}

void updateLiftPID (float error) {
    float dT = AUTON_LOOP_DELAY;
    computePID(&liftPID,error,dT);
    setLiftSpeed(liftPID.output);
}

void initializeClawPID(float kp, float ki, float kd) {
    intializePID(&clawPID,kp,ki,kd);
}

void updateClawPID (float error) {
    float dT = AUTON_LOOP_DELAY;
    computePID(&clawPID,error,dT);
    setClawSpeed(clawPID.output);
}

//---Autonomous Loop Methods---//

void runWaypoint(waypoint * wp) {
	bool isComplete = false;
	initializeSensors();
	writeDebugStreamLine("RUNNING WAYPOINT WITH VALS FOW:%f, STRF:%f, LIFT:%f, ROT:%f, CLAW:%f",wp->forwardTicks,wp->strafeTicks,wp->rotationAngle,wp->liftPosition,wp->clawPosition);

	while (!isComplete) {

		isComplete = true;

		//make sure these errors are in the right direction!!!
		float driveForwardsErrorL = (wp->forwardTicks) -  SensorValue[DRIVE_LEFT_BACK_QUAD];
		float driveForwardsErrorR = (wp->forwardTicks) - -SensorValue[DRIVE_RIGHT_BACK_QUAD];
		float rotationError = (wp->rotationAngle) - SensorValue[DRIVE_GYRO];
		float liftError = map((wp->liftPosition),0,100,LIFT_POT_VALUE_MIN,LIFT_POT_VALUE_MAX)-SensorValue[LIFT_POTENTIOMETER];
		float clawError = map((wp->clawPosition),0,100,CLAW_CLOSED_POT_VALUE,CLAW_OPENED_POT_VALUE)-SensorValue[CLAW_POTENTIOMETER];
		writeDebugStreamLine("WAYPOINT ERRORS ARE DRIVE: %f ROT: %f LIFT: %f CLAW: %f",driveForwardsErrorL,rotationError,liftError,clawError);

		//Move forwards or backwards until error is below threshold
		if(fabs(driveForwardsErrorL) > DRIVE_FORWARDS_ERROR_THRESH || fabs(driveForwardsErrorR) > DRIVE_FORWARDS_ERROR_THRESH) {
			isComplete = false;
			float speed = linearSpeedCurve((driveForwardsErrorL+driveForwardsErrorR)/2,AUTON_FORWARDS_DECEL_DIST);
			writeDebugStream("RUNNING DRIVE FORWARDS AT SPEED %f",speed);
			updateDrivePID(speed,speed);
		}

		//Rotate left or right until the error is below threshold
		else if(fabs(rotationError) > DRIVE_ROTATION_ERROR_THRESH) {
			isComplete = false;
			float speed = linearSpeedCurve(rotationError,AUTON_ROTATION_DECEL_DIST);
			writeDebugStreamLine("TURNING DRIVE AT SPEED %f",speed);
			updateDrivePID(speed,-speed);
		}

		updateLiftPID(liftError);
		if(fabs(liftError) > LIFT_ERROR_THRESH) {
			isComplete = false;
		}

		updateClawPID(clawError);
		if(fabs(clawError) > CLAW_ERROR_THRESH) {
			isComplete = false;
		}

		wait1Msec(AUTON_LOOP_DELAY);
	}
}

//TODO: add defaults into constants file
void initializeDefaultWaypoint(waypoint* wp) {
	wp->forwardTicks = 0;
	wp->strafeTicks = 0;
	wp->rotationAngle = 0;
	wp->liftPosition = 10;
	wp->clawPosition = 10;

}

void initializeWaypointArray() {
//drive code routine goes here
	for (int i =0; i<NUM_WAYPOINTS; i++) {
		writeDebugStreamLine("INITIALIZING WAYPOING %d",i);
		initializeDefaultWaypoint(&waypointsList[i]);
	}
	waypointsList[0].forwardTicks = 2104;
}

void runAutonomousLoop() {
	initializeWaypointArray();
	initializeDrivePID(DRIVE_PID_KP,DRIVE_PID_KI,DRIVE_PID_KD);
	initializeClawPID(CLAW_PID_KP,CLAW_PID_KI,CLAW_PID_KD);
	initializeLiftPID(LIFT_PID_KP,LIFT_PID_KI,LIFT_PID_KD);

	for (int i =0; i<NUM_WAYPOINTS;i++) {
		runWaypoint(&waypointsList[i]);
	}

	//Example Autonomous Routine
	//move forwards
	//turn left
	//move forward
	//turn right
	//lift arm
	//open claw
	//move forward
	//move back
	//lower arm
	//turn right
	//move forward
	//turn left
	//lift arm
	//move forward
	//move backward
	//lower arm
	//turn right
	//move forward
	//turn left
	//lift arm
	//move forward
	//move backward
	//lower arm

}
