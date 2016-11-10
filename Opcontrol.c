

void runOpcontrolLoop() {
    nMotorEncoder[LEFT_MOTOR_ENCODER] = 0;
    nMotorEncoder[RIGHT_MOTOR_ENCODER] = 0;
    
    while (true)
    {
        
        if (vexRT[Btn7U] == 1) {
            setLiftSpeed(LIFT_HIGH_SPEED);
        }
        else if (vexRT[Btn7D]==1) {
            setLiftSpeed(-LIFT_HIGH_SPEED);
        }
        else if (vexRT[Btn7L] == 1) {
            setLiftSpeed(0);
        }
        else if (vexRT[Btn7R]==1) {
            setLiftSpeed(0);
        }
        else {
            runLiftControlLoop(liftState);
        }
        
        if(vexRT[Btn6U] == 1){
            setClawSpeed(CLAW_OPEN_SPEED);
        }
        else if(vexRT[Btn6D] == 1){
            setClawSpeed(CLAW_CLOSE_SPEED);
        }
        else{
            setClawSpeed(0);
        }
        
        //SET THE LIFT STATE
        if(vexRT[Btn5U] == 1){
            liftState = 1;
        }
        else if (vexRT[Btn5D] == 1){
            liftState = 0;
        }
        
        
        //Drive Control
        int frontRightMotorSpeed = -((vexRT[Ch3] - vexRT[Ch4]) - vexRT[Ch1]);
        int backRightMotorSpeed = -((vexRT[Ch3] - vexRT[Ch4]) + vexRT[Ch1]);
        int frontLeftMotorSpeed = ((vexRT[Ch3] + vexRT[Ch4]) + vexRT[Ch1]);
        int backLeftMotorSpeed = ((vexRT[Ch3] + vexRT[Ch4]) - vexRT[Ch1]);
        
        
        motor[FRONT_RIGHT_DRIVE_MOTOR] = frontRightMotorSpeed;
        motor[BACK_RIGHT_DRIVE_MOTOR] = backRightMotorSpeed;
        motor[FRONT_LEFT_DRIVE_MOTOR] = frontLeftMotorSpeed;
        motor[BACK_LEFT_DRIVE_MOTOR] = backLeftMotorSpeed;
        
        
    }
}
