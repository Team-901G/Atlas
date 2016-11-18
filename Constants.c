//OP CONTROL CONSTANTS
int LIFT_HIGH_SPEED = 127;
int LIFT_LOW_SPEED = 64;
int CLAW_OPEN_SPEED = 128;
int CLAW_CLOSE_SPEED = -70;

//LIFT CONTROL CONSTANTS
int LIFT_UP_SPEED = 100;
int LIFT_DOWN_SPEED = -60;
int LIFT_POT_VALUE_MAX = 2000;
int LIFT_POT_VALUE_MIN = 366;
int LIFT_POT_VALUE_HIGH= LIFT_POT_VALUE_MAX - 150;
int LIFT_POT_VALUE_LOW = LIFT_POT_VALUE_MIN + 100;
int LIFT_DOWN_HOLDING_SPEED = 0;
int LIFT_UP_HOLDING_SPEED = 40;

//CLAW CONTROL CONSTANTS
int CLAW_OPENED_POT_VALUE = 3500; //totally open -- 180 degrees
int CLAW_CLOSED_POT_VALUE= 1730;

//AUTONOMOUS CONTROL CONSTANTS
float DIFF_DRIVE_PID_KP = 0;//-0.8;//positive means veer left (comp for right shift)
float DIFF_DRIVE_PID_KI = 0;
float DIFF_DRIVE_PID_KD = 0;

float DRIVE_FORWARD_PID_KP = 0.6;
float DRIVE_FORWARD_PID_KI = 0; //probably no I here
float DRIVE_FORWARD_PID_KD = 0;

float DRIVE_ROTATION_PID_KP = 0.6;
float DRIVE_ROTATION_PID_KI = 0; //probably no I here
float DRIVE_ROTATION_PID_KD = 0;

float LIFT_PID_KP = 1.2;//((2000-366)/100)* 0.1;//error is usuallt around 1500 -> 30 default speed
float LIFT_PID_KI = 0.000035;//0.00001;//((2000-366)/100)* 0.0002; //1500 error ticks accumulated per sec -> +7.5 speed after 1 seconds
float LIFT_PID_KD = 0;

float OP_LIFT_PID_KP = 0.1;
float OP_LIFT_PID_KI = 0.0002;
float OP_LIFT_PID_KD = 0;

float CLAW_PID_KP = 1.5;//0.1;
float CLAW_PID_KI = 0;//0.1;
float CLAW_PID_KD = 0;//0.5;

float CLAW_PID_CONTROL_POS_THRESH = 20; //if above this, use pid

float DRIVE_FORWARD_ERROR_THRESH = 20;//50 //about 5cm
float DRIVE_ROTATION_ERROR_THRESH = 50;//50 //5 degrees, might be too small
float CLAW_ERROR_THRESH = 20;//100 // in potentiometer ticks
float LIFT_ERROR_THRESH = 30; //50 in potentiometer ticks

int AUTON_LOOP_DELAY = 30;
int AUTON_DRIVE_MAX_SPEED = 110;
int AUTON_WAYPOINT_NOERROR_TICKS = 50; //how long the waypoint must be with no error before proceeding to next wp

//60 cm is 647 651 677 675 avg 663
//1 meter is 1104 ticks
