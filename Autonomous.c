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

PIDObject diffDrivePID;
PIDObject drivePID;
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

float speedMax(float val, float max) {
	return abs(val) > abs(max) ? sign(val)*max : val;
}
//---PID FUNCTIONS---//

/*float linearSpeedCurve(float remaining, float threshold) {
	if (fabs(remaining) > threshold) {
		return sign(remaining)*AUTON_DRIVE_MAX_SPEED;
		}
	else {
		return (remaining / threshold)* AUTON_DRIVE_MAX_SPEED;
	}
}*/

//should work for forwards, backwards, left turn, and right turn movement
//this only control driving straight -- right now the linearSpeedCurve acts as a P(no I no D) controller
//might want to change linearSpeedCurve into a PID controller
void updateDiffDrivePID(int leftSpeed, int rightSpeed) {
    float dT = AUTON_LOOP_DELAY;

    float frontLeftTicksRemaining = - -SensorValue[DRIVE_LEFT_FRONT_QUAD];
    float backLeftTicksRemaining = - SensorValue[DRIVE_LEFT_BACK_QUAD];
    float leftAvgTicksRemaining = (frontLeftTicksRemaining + backLeftTicksRemaining)/2;

    float frontRightTicksRemaining =  - SensorValue[DRIVE_RIGHT_FRONT_QUAD];
    float backRightTicksRemaining = - -SensorValue[DRIVE_RIGHT_BACK_QUAD];
    float rightAvgTicksRemaining = (frontRightTicksRemaining + backRightTicksRemaining)/2;

    float tickDiff = fabs(rightAvgTicksRemaining) - fabs(leftAvgTicksRemaining);
    computePID(&diffDrivePID,tickDiff,dT);

    //left side should compensate if right side is going faster (kP is positive)
    int leftOutputSpeed = sign(leftAvgTicksRemaining)*((int)(diffDrivePID.output))+leftSpeed;
    int rightOutputSpeed = rightSpeed;

    setDriveSpeed(leftOutputSpeed,rightOutputSpeed,leftOutputSpeed,rightOutputSpeed);
}

float updateDrivePID (float error) {
		float dT = AUTON_LOOP_DELAY;
		computePID(&drivePID,error,dT);
		return drivePID.output;
}

float updateRotationPID (float error) {
		float dT = AUTON_LOOP_DELAY;
		computePID(&rotationPID,error,dT);
		return rotationPID.output;
}

float updateLiftPID (float error) {
    float dT = AUTON_LOOP_DELAY;
    computePID(&liftPID,error,dT);
    return liftPID.output;
}

float updateClawPID (float error) {
    float dT = AUTON_LOOP_DELAY;
    computePID(&clawPID,error,dT);
    return clawPID.output;
}

//---Autonomous Loop Methods---//

void runWaypoint(waypoint * wp) {
	bool isComplete = false;
	bool driveComplete = false;
	initializeSensors();
	writeDebugStreamLine("RUNNING WAYPOINT WITH VALS FOW:%f, STRF:%f, LIFT:%f, ROT:%f, CLAW:%f",wp->forwardTicks,wp->strafeTicks,wp->rotationAngle,wp->liftPosition,wp->clawPosition);

	while (!isComplete) {

		isComplete = true;

		//make sure these errors are in the right direction!!!
		float driveForwardsErrorL = (wp->forwardTicks) -  SensorValue[DRIVE_LEFT_BACK_QUAD];
		float driveForwardsErrorR = (wp->forwardTicks) - -SensorValue[DRIVE_RIGHT_BACK_QUAD];
		float driveForwardsError = (driveForwardsErrorL+driveForwardsErrorR)/2;
		float rotationError = (wp->rotationAngle) - SensorValue[DRIVE_GYRO];
		float liftError = map((wp->liftPosition),0,100,LIFT_POT_VALUE_MIN,LIFT_POT_VALUE_MAX)-SensorValue[LIFT_POTENTIOMETER];
		float clawError = map((wp->clawPosition),0,100,CLAW_CLOSED_POT_VALUE,CLAW_OPENED_POT_VALUE)-SensorValue[CLAW_POTENTIOMETER];
		writeDebugStreamLine("WAYPOINT ERRORS ARE DRIVE: %f ROT: %f LIFT: %f CLAW: %f",driveForwardsErrorL,rotationError,liftError,clawError);
		datalogAddValue(0,driveForwardsError);
		datalogAddValue(1,rotationError);
		datalogAddValue(2,liftError);
		datalogAddValue(3,clawError);


		//Move forwards or backwards until error is below threshold
		if(!driveComplete && (fabs(driveForwardsErrorL) > DRIVE_FORWARDS_ERROR_THRESH || fabs(driveForwardsErrorR) > DRIVE_FORWARDS_ERROR_THRESH)) {
			isComplete = false;
			float speed = updateDrivePID(driveForwardsError);
			speed = speedMax(speed,AUTON_DRIVE_MAX_SPEED);
			writeDebugStream("RUNNING DRIVE FORWARDS AT SPEED %f",speed);
			updateDiffDrivePID(speed,speed);
		}
		//Rotate left or right until the error is below threshold
		else if(fabs(rotationError) > DRIVE_ROTATION_ERROR_THRESH) {
			driveComplete = true;
			isComplete = false;
			float speed = updateRotationPID(rotationError);
			speed = speedMax(speed,AUTON_DRIVE_MAX_SPEED);
			writeDebugStreamLine("TURNING DRIVE AT SPEED %f",speed);
			updateDiffDrivePID(speed,-speed);
		}

		setLiftSpeed(updateLiftPID(liftError));
		if(fabs(liftError) > LIFT_ERROR_THRESH) {
			isComplete = false;
		}

		setClawSpeed(updateClawPID(clawError));
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
	//waypointsList[0].forwardTicks = 2200;
	//waypointsList[0].rotationAngle = 0;
	waypointsList[0].clawPosition = 50;
	waypointsList[1].clawPosition = 90;
	waypointsList[2].clawPosition = 10;
}

void runAutonomousLoop() {
	initializeWaypointArray();
	initializePID(&diffDrivePID,DIFF_DRIVE_PID_KP,DIFF_DRIVE_PID_KI,DIFF_DRIVE_PID_KD);
	initializePID(&drivePID,DRIVE_FORWARDS_PID_KP,DRIVE_FORWARDS_PID_KI,DRIVE_FORWARDS_PID_KD);
	initializePID(&rotationPID,DRIVE_ROTATION_PID_KP,DRIVE_ROTATION_PID_KI,DRIVE_ROTATION_PID_KD);
	initializePID(&clawPID,CLAW_PID_KP,CLAW_PID_KI,CLAW_PID_KD);
	initializePID(&liftPID,LIFT_PID_KP,LIFT_PID_KI,LIFT_PID_KD);

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
