//OP CONTROL CONSTANTS
int LIFT_HIGH_SPEED = 127;
int LIFT_LOW_SPEED = 64;
int CLAW_OPEN_SPEED = 128;
int CLAW_CLOSE_SPEED = -70;

//LIFT CONTROL CONSTANTS
int LIFT_UP_SPEED = 128;
int LIFT_DOWN_SPEED = -60;
int LIFT_POTENTIOMETER_VALUE_MAX = 2000;
int LIFT_POTENTIOMETER_VALUE_MIN = 366;
int LIFT_POTENTIOMETER_VALUE_HIGH= LIFT_POTENTIOMETER_VALUE_MAX - 150;
int LIFT_POTENTIOMETER_VALUE_LOW = LIFT_POTENTIOMETER_VALUE_MIN + 100;
int LIFT_DOWN_HOLDING_SPEED = 0;
int LIFT_UP_HOLDING_SPEED = 50;

//AUTONOMOUS CONTROL CONSTANTS
float DRIVE_PID_KP = 0.1;
float DRIVE_PID_KI = 0;
float DRIVE_PID_KD = 0;

float LIFT_PID_KP = 0;
float LIFT_PID_KI = 0;
float LIFT_PID_KD = 0;

float CLAW_PID_KP = 0;
float CLAW_PID_KI = 0;
float CLAW_PID_KD = 0;

float DRIVE_FORWARDS_ERROR_THRESH = 50; //about 5cm
float DRIVE_ROTATION_ERROR_THRESH = 50; //5 degrees, might be too small
float CLAW_ERROR_THRESH = 100;
float LIFT_ERROR_THRESH = 50;

int AUTON_DRIVE_MAX_SPEED = 110;
int AUTON_FORWARDS_DECEL_DIST = 200; //when to start const decel
int AUTON_ROTATION_DECEL_DIST = 300; //300 = 30 degrees (3600 is full rev)

//60 cm is 647 651 677 675 avg 663
//1 meter is 1104 ticks
