typedef struct {
   float kP,kI,kD;
   float error, integral, derivative, pastError;
   float target;
   float output;
} PIDObject ;


void initializePID(PIDObject* pid, float kP,float kI,float kD) {
        pid -> kP = kP;
        pid -> kI = kI;
        pid -> kD = kD;
        pid -> error = 0;
        pid -> integral = 0;
        pid -> derivative = 0;
        pid -> pastError = 0;
        pid -> output = 0;
}

void computePID(PIDObject* pid, float error, float dT) {
        pid -> pastError = pid-> error;
        pid -> error = error;
        pid -> integral += error*dT;
        pid -> derivative = (pid->error - pid->pastError)/dT;

        pid -> output = (pid->kP*pid->error+pid->kI*pid->integral+pid->kD*pid->derivative);
}
