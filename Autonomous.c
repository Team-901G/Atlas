//Convention: a positive turn angle means turning right
//a positive remaining distance implies a positive speed needs to be applied to close the gap
//this is assuming right rotation is positive on the gyro

//IMPLEMENT ROTATION PID SO IT USES DRIVE PID
//TUNE PID CONSTANTS
//IMPLEMENT ROUTINE
typedef struct {
    float forwardTicks;     //in quad encoder ticks delta
    float strafeTicks;      //in quad encoder ticks delta
    float rotationAngle;    //in gyro angle delta
    float liftPosition;     //in potentiometer value
    float clawPosition;     //in potentiometer value

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


//---PID FUNCTIONS---//

float linearSpeedCurve(float remaining, float threshold) {
	if (abs(remaining) > threshold) {
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
    float dT = 0.01;//maybe store the last time in the PIDObjecth

    float frontLeftTicksRemaining = - -SensorValue[DRIVE_LEFT_FRONT_QUAD];
    float backLeftTicksRemaining = - SensorValue[DRIVE_LEFT_BACK_QUAD];
    float leftAvgTicksRemaining = (frontLeftTicksRemaining + backLeftTicksRemaining)/2;

    float frontRightTicksRemaining =  - SensorValue[DRIVE_RIGHT_FRONT_QUAD];
    float backRightTicksRemaining = - -SensorValue[DRIVE_RIGHT_BACK_QUAD];
    float rightAvgTicksRemaining = (frontRightTicksRemaining + backRightTicksRemaining)/2;

    float tickDiff = abs(rightAvgTicksRemaining) - abs(leftAvgTicksRemaining);
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

void updateLiftPID (int position) {
    float dT = 0.01;
    computePID(&liftPID,SensorValue[LIFT_POTENTIOMETER]-position,dT);
    setLiftSpeed(liftPID.output);
}

void initializeClawPID(float kp, float ki, float kd) {
    intializePID(&clawPID,kp,ki,kd);
}

void updateClawPID (int position) {
    float dT = 0.01;
    computePID(&clawPID,SensorValue[CLAW_POTENTIOMETER]-position,dT);
    setClawSpeed(clawPID.output);
}

//---Autonomous Loop Methods---//

void runWaypoint(waypoint * wp) {
	bool isComplete = false;
	initializeSensors();
	writeDebugStreamLine("RUNNING WAYPOINT WITH VALS %f, %f, %f, %f, %f",wp->forwardTicks,wp->strafeTicks,wp->rotationAngle,wp->liftPosition,wp->clawPosition);

	while (!isComplete) {

		isComplete = true;

		//make sure these errors are in the right direction!!!
		float driveForwardsErrorL = (wp->forwardTicks) -  SensorValue[DRIVE_LEFT_BACK_QUAD];
		float driveForwardsErrorR = (wp->forwardTicks) - -SensorValue[DRIVE_RIGHT_BACK_QUAD];
		float rotationError = (wp->rotationAngle) - SensorValue[DRIVE_GYRO];

		//Move forwards or backwards until error is below threshold
		if(abs(driveForwardsErrorL) > DRIVE_FORWARDS_ERROR_THRESH || abs(driveForwardsErrorR) > DRIVE_FORWARDS_ERROR_THRESH) {
			isComplete = false;
			float speed = linearSpeedCurve((driveForwardsErrorL+driveForwardsErrorR)/2,AUTON_FORWARDS_DECEL_DIST);
			updateDrivePID(speed,speed);
		}

		//Rotate left or right until the error is below threshold
		else if(abs(rotationError) > DRIVE_ROTATION_ERROR_THRESH) {
			isComplete = false;
			float speed = linearSpeedCurve(rotationError,AUTON_ROTATION_DECEL_DIST);
			updateDrivePID(speed,-speed);
		}

		wait1Msec(20);
	}
}

//IN TERMS OF A DELTA
void initializeDefaultWaypoint(waypoint* wp) {
	wp->forwardTicks = 0;
	wp->strafeTicks = 0;
	wp->rotationAngle = 0;
	wp->liftPosition = 0;
	wp->clawPosition = 0;

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

	for (int i =0; i<NUM_WAYPOINTS;i++) {
		runWaypoint(&waypointsList[i]);
	}

	//def
	//moveForward(100,70)
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
