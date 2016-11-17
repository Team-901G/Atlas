//Convention: a positive turn angle means turning right
//a positive remaining distance implies a positive speed needs to be applied to close the gap
//this is assuming right rotation is positive on the gyro
//also assuming that claw opens with positive value to setClawSpeed, and pot gets bigger with claw more open

typedef struct {
    float forwardTicks;     //in quad encoder ticks delta
    float strafeTicks;      //in quad encoder ticks delta
    float rotationAngle;    //in gyro angle delta
    float liftPosition;     //from 0 to 100, 0 is down
    float clawPosition;     //from 0 to 100, 0 is closed
		float waitTime; 				//wait time after completion
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

float clawSpeedCurve(float error) {
    float dT = AUTON_LOOP_DELAY;
    if(abs(error)<20) {
      return sign(error) * CLAW_HOLDING_SPEED;
    }
    else{
      return sign(error) * CLAW_MOVING_SPEED;
    }

}

//---Autonomous Loop Methods---//

void runWaypoint(waypoint * wp) {
	bool isComplete = false;
	bool driveComplete = false;
  int numTicksComplete = 0;
	initializeSensors();
	writeDebugStreamLine("RUNNING WAYPOINT WITH VALS FOW:%f, STRF:%f, LIFT:%f, ROT:%f, CLAW:%f",wp->forwardTicks,wp->strafeTicks,wp->rotationAngle,wp->liftPosition,wp->clawPosition);

	while (!isComplete && numTicksComplete < AUTON_WAYPOINT_NOERROR_TICKS) {

		//isComplete = true;

		//make sure these errors are in the right direction!!!
		float driveForwardsErrorL = (wp->forwardTicks) -  SensorValue[DRIVE_LEFT_BACK_QUAD];
		float driveForwardsErrorR = (wp->forwardTicks) - -SensorValue[DRIVE_RIGHT_BACK_QUAD];
		float driveForwardsError = (driveForwardsErrorL+driveForwardsErrorR)/2;
		float rotationError = (wp->rotationAngle) - SensorValue[DRIVE_GYRO];
		float liftError = (wp->liftPosition)-map(SensorValue[LIFT_POTENTIOMETER],LIFT_POT_VALUE_MIN,LIFT_POT_VALUE_MAX,0,100);
		float clawError = (wp->clawPosition)-map(SensorValue[CLAW_POTENTIOMETER],CLAW_CLOSED_POT_VALUE,CLAW_OPENED_POT_VALUE,0,100);
		//writeDebugStreamLine("WP ERRORS: DR:%d RT:%d L:%d CL: %d",(int)(driveForwardsErrorL),(int)(rotationError),(int)(liftError),(int)(clawError));
		datalogAddValue(0,driveForwardsError);
		datalogAddValue(1,rotationError);
		datalogAddValue(2,liftError);
		datalogAddValue(3,clawError);
    datalogAddValue(4,diffDrivePID.error);

    datalogAddValue(5,drivePID.derivative);
    datalogAddValue(6,rotationPID.derivative);
    datalogAddValue(7,liftPID.derivative);
    datalogAddValue(8,diffDrivePID.derivative);

		//Move forwards or backwards until error is below threshold
		if(!driveComplete && (fabs(driveForwardsErrorL) > DRIVE_FORWARD_ERROR_THRESH || fabs(driveForwardsErrorR) > DRIVE_FORWARD_ERROR_THRESH)) {
			isComplete = false;
			float speed = updateDrivePID(driveForwardsError);
			speed = speedMax(speed,AUTON_DRIVE_MAX_SPEED);
			//writeDebugStream("RUNNING DRIVE FORWARDS AT SPEED %d",(int)speed);
      datalogAddValue(9,(int)speed)
      updateDiffDrivePID(speed,speed);
		}
		//Rotate left or right until the error is below threshold
		else if(fabs(rotationError) > DRIVE_ROTATION_ERROR_THRESH) {
			driveComplete = true;
			isComplete = false;
			float speed = updateRotationPID(rotationError);
			speed = speedMax(speed,AUTON_DRIVE_MAX_SPEED);
			//writeDebugStreamLine("TURNING DRIVE AT SPEED %d",(int)speed);
      datalogAddValue(10,(int)speed);
      updateDiffDrivePID(speed,-speed);
		}

		float liftSpeed = updateLiftPID(liftError);
		setLiftSpeed(liftSpeed);
		//writeDebugStreamLine("RUNNING LIFT AT SPEED %d",(int)liftSpeed);
    datalogAddValue(11,(int)liftSpeed)
    if(fabs(liftError) > LIFT_ERROR_THRESH) {
			isComplete = false;
		}

    float clawSpeed = 0;
    if (wp->clawPosition > CLAW_PID_CONTROL_POS_THRESH) {
		    clawSpeed = updateClawPID(clawError);
		    setClawSpeed(clawSpeed);
      }
    else {
        clawSpeed = clawSpeedCurve(clawError);
        setClawSpeed(clawSpeed);
    }
		//writeDebugStreamLine("RUNNING CLAW AT SPEED %d",(int)clawSpeed);
    datalogAddValue(12,(int)clawSpeed);
    if(fabs(clawError) > CLAW_ERROR_THRESH) {
			isComplete = false;
		}

    if(isComplete == false) {
      numTicksComplete = 0;
    }
    else {
      numTicksComplete ++;
    }

		wait1Msec(AUTON_LOOP_DELAY);
	}
	wait1Msec(wp->waitTime);
}

//TODO: add defaults into constants file
void initializeDefaultWaypoint(waypoint* wp) {
	wp->forwardTicks = 0;
	wp->strafeTicks = 0;
	wp->rotationAngle = 0;
	wp->liftPosition = 6;
	wp->clawPosition = 10;
  wp->waitTime = 0;
}

void initializeWaypointArray() {
//drive code routine goes here
	for (int i =0; i<NUM_WAYPOINTS; i++) {
		writeDebugStreamLine("INITIALIZING WAYPOING %d",i);
		initializeDefaultWaypoint(&waypointsList[i]);
	}

  //TODO: can create and reuuse waypoints that do specific things by making a function that sets them to that
  waypointsList[0].liftPosition = 30;

  //First Cube
  waypointsList[1].liftPosition = 11;
  waypointsList[1].clawPosition = 90;
  waypointsList[1].waitTime = 3000;

  waypointsList[2].clawPosition = 10;

  waypointsList[3].liftPosition=90;
  waypointsList[3].forwardTicks = 500;

  waypointsList[4].liftPosition = 90;
  waypointsList[4].clawPosition = 90;
  waypointsList[4].waitTime = 1000;

  waypointsList[5].liftPosition = 90;
  waypointsList[5].forwardTicks = -500;
  waypointsList[5].clawPosition = 90;

  //Second Cube
  waypointsList[6].liftPosition = 11;
  waypointsList[6].clawPosition = 90;
  waypointsList[6].waitTime = 3000;

  waypointsList[7].clawPosition = 10;

  waypointsList[8].liftPosition=90;
  waypointsList[8].forwardTicks = 500;

  waypointsList[9].liftPosition = 90;
  waypointsList[9].clawPosition = 90;
  waypointsList[9].waitTime = 1000;

  //Returning and TURNING

  waypointsList[10].liftPosition = 90;
  waypointsList[10].rotationAngle = 180;
  waypointsList[10].clawPosition = 90;

  waypointsList[11].liftPosition = 11;
  waypointsList[11].clawPosition = 90;
  waypointsList[11].waitTime = 3000;

  //Grabbing stars and dumping over fence
  waypointsList[12].clawPosition=11;

  waypointsList[13].liftPosition = 90;

  waypointsList[14].liftPosition = 90;
  waypointsList[14].rotationAngle = 180;

  waypointsList[15].clawPosition = 90;
  waypointsList[15].liftPosition = 90;
  waypointsList[15].waitTime = 1000;

  //turning 90. move fow, and grabbing Cube, and moving forward, and turning right
  waypointsList[16].rotationAngle = 90;
  waypointsList[16].clawPosition = 90;

  waypointsList[17].liftPosition = 11;
  waypointsList[17].clawPosition = 90;

  waypointsList[18].forwardTicks = 200;
  waypointsList[18].clawPosition = 90;

  waypointsList[19].clawPosition = 11;

  waypointsList[20].liftPosition = 90;
  waypointsList[20].clawPosition = 11;
  waypointsList[20].forwardTicks = 200;
  waypointsList[20].rotationAngle = -90;

  //move forwards and drop Cube
  waypointsList[21].liftPosition = 90;
  waypointsList[21].forwardTicks = 200;

  waypointsList[22].clawPosition = 90;
  waypointsList[22].waitTime = 1000;
}

void runAutonomousLoop() {
	initializeWaypointArray();
	initializePID(&diffDrivePID,DIFF_DRIVE_PID_KP,DIFF_DRIVE_PID_KI,DIFF_DRIVE_PID_KD);
	initializePID(&drivePID,DRIVE_FORWARD_PID_KP,DRIVE_FORWARD_PID_KI,DRIVE_FORWARD_PID_KD);
	initializePID(&rotationPID,DRIVE_ROTATION_PID_KP,DRIVE_ROTATION_PID_KI,DRIVE_ROTATION_PID_KD);
	initializePID(&clawPID,CLAW_PID_KP,CLAW_PID_KI,CLAW_PID_KD);
	initializePID(&liftPID,LIFT_PID_KP,LIFT_PID_KI,LIFT_PID_KD);

	for (int i =0; i<NUM_WAYPOINTS;i++) {
		runWaypoint(&waypointsList[i]);
	}

}

void runOpcontrolLoop() {
    initializeSensors();
    initializePID(&liftPID,LIFT_PID_KP,LIFT_PID_KI,LIFT_PID_KD);
    int liftTarget = 11;

    while (true)
    {
				//Run lift using PID:
    		float liftError = map(liftTarget,0,100,LIFT_POT_VALUE_MIN,LIFT_POT_VALUE_MAX)-SensorValue[LIFT_POTENTIOMETER];
    		float liftSpeed = updateLiftPID(liftError);
				setLiftSpeed(liftSpeed);

        if (vexRT[Btn7U] == 1) {
        		//setLiftSpeed(LIFT_HIGH_SPEED);
        }
        else if (vexRT[Btn7D]==1) {
        		//setLiftSpeed(-LIFT_HIGH_SPEED);
        }
        else if (vexRT[Btn7L] == 1) {
            setLiftSpeed(0);
        }
        else if (vexRT[Btn7R]==1) {
            setLiftSpeed(0);
        }
        else {
            //runLiftControlLoop(liftState);
        }
        if(vexRT[Btn5U] == 1){
           liftTarget = 90;
        }
        else if (vexRT[Btn5D] == 1){
           liftTarget = 11;
        }

        //Claw Control
        if(vexRT[Btn6U] == 1){
            setClawSpeed(CLAW_OPEN_SPEED);
        }
        else if(vexRT[Btn6D] == 1){
            setClawSpeed(CLAW_CLOSE_SPEED);
        }
        else{
            setClawSpeed(0);
        }

        //Drive Control
        int frontRightMotorSpeed = - -((vexRT[Ch3] - vexRT[Ch4]) - vexRT[Ch1]);
        int backRightMotorSpeed = - -((vexRT[Ch3] - vexRT[Ch4]) + vexRT[Ch1]);
        int frontLeftMotorSpeed = ((vexRT[Ch3] + vexRT[Ch4]) + vexRT[Ch1]);
        int backLeftMotorSpeed = ((vexRT[Ch3] + vexRT[Ch4]) - vexRT[Ch1]);

				setDriveSpeed(frontLeftMotorSpeed,frontRightMotorSpeed,backLeftMotorSpeed,backRightMotorSpeed);

    }
}
