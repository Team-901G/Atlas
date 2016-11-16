//TODO: ADD WAYPOINT ARRAY ITERATOR / RUNNER FUNCTION
//ADD WAYPOINT ARRAY
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

PIDObject frontLeftDrivePID;
PIDObject frontRightDrivePID;
PIDObject backLeftDrivePID;
PIDObject backRightDrivePID;
PIDObject rotationPID;
PIDObject liftPID;
PIDObject clawPID;

void initializeWaypointArray() {
//drive code routine goes here

}

void initializeDrivePID() {
    intializePID(frontLeftDrivePID,0,0,0);
    intializePID(frontRightDrivePID,0,0,0);
    intializePID(backLeftDrivePID,0,0,0);
    intializePID(backRightDrivePID,0,0,0);
}

void updateDrivePID(int frontLeft,int frontRight, int backLeft, int backRight) {
    float dT = 0;//maybe store the last time in the PIDObjecth
    computePID(frontLeftDrivePID,SensorValue[DRIVE_LEFT_FRONT_QUAD]-frontLeft,dT);
    computePID(frontRightDrivePID,SensorValue[DRIVE_RIGHT_FRONT_QUAD]-frontRight,dT);
    computePID(backLeftDrivePID,SensorValue[DRIVE_LEFT_BACK_QUAD]-backLeft,dT);
    computePID(backRightDrivePID,SensorValue[DRIVE_RIGHT_BACK_QUAD]-backRight,dT);
    setDriveSpeed(frontLeftDrivePID->output,frontRightDrivePID->output,backLeftDrivePID->output,backRightDrivePID->output)l
}

void initializeDrivePID() {
   initializePID(rotationPID,0,0,0); 
}

void initializeLiftPID() {
    initializePID(liftPID,0,0,0);
}

void updateLiftPID (int position) {
    float dT = 0;
    computePID(liftPID,SensorValue[LIFT_POTENTIOMETER]-position,dT);
    setLiftSpeed(liftPID->output);
}

void initializeClawPID() {
    initalizePID(clawPID,0,0,0);
}

void updateClawPID (int position) {
    float dT = 0;
    computePID(clawPID,SensorValue[CLAW_POTENTIOMETER]-position,dT);
    setClawSpeed(clawPID->output);
}



void runAutonomousLoop() {
	//def
	moveForward(100,70)
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
