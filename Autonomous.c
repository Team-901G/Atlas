
//ADD CONSTANTS FOR EACH PID IN CONSTANTS.C
//IMPLEMENT ROTATION PID SO IT USES DRIVE PID
//TUNE PID CONSTANTS
//IMPLEMENT ROUTINE
typedef struct {
    float forwardTicks;     //in quad encoder ticks
    float strafeTicks;      //in quad encoder ticks
    float rotationAngle;    //in gyro angle delta
    float liftPosition;     //in potentiometer value
    float clawPosition;     //in potentiometer value

} waypoint;


waypoint waypointsList[10];
int NUM_WAYPOINTS = 10;

PIDObject differentialDrivePID;
PIDObject frontLeftDrivePID;
PIDObject frontRightDrivePID;
PIDObject backLeftDrivePID;
PIDObject backRightDrivePID;
PIDObject rotationPID;
PIDObject liftPID;
PIDObject clawPID;


//---PID FUNCTIONS---//

void initializeDrivePID(float kp, float ki, float kd) {
    intializePID(&frontLeftDrivePID,kp,ki,kd);
    intializePID(&frontRightDrivePID,kp,ki,kd);
    intializePID(&backLeftDrivePID,kp,ki,kd);
    intializePID(&backRightDrivePID,kp,ki,kd);
}

void updateDrivePID(int frontLeft,int frontRight, int backLeft, int backRight) {
    float dT = 0.01;//maybe store the last time in the PIDObjecth
    computePID(&frontLeftDrivePID,frontLeft- -SensorValue[DRIVE_LEFT_FRONT_QUAD],dT);
    computePID(&frontRightDrivePID,frontRight - SensorValue[DRIVE_RIGHT_FRONT_QUAD],dT);
    computePID(&backLeftDrivePID,backLeft - SensorValue[DRIVE_LEFT_BACK_QUAD],dT);
    computePID(&backRightDrivePID,backRight- -SensorValue[DRIVE_RIGHT_BACK_QUAD],dT);
    setDriveSpeed(frontLeftDrivePID.output,frontRightDrivePID.output,backLeftDrivePID.output,backRightDrivePID.output);
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
		if((abs(SensorValue[DRIVE_LEFT_BACK_QUAD]) - fabs(wp->forwardTicks))>DRIVE_FOWARDTICKS_ERROR_THRESH) {
			isComplete = false;
			updateDrivePID(wp->forwardTicks,wp->forwardTicks,wp->forwardTicks,wp->forwardTicks);
		}
		wait1Msec(10);
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
