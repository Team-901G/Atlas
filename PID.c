typedef struct {
   float kP,kI,kD;
   float error, integral, derivative, pastError; 
   float target;
   float output;
} PIDObject ;


void intializePID(PIDObject* pid, float kP,float kI,float kD) {
        pid -> kP = kP;
        pid -> kI = kI;
        pid -> kD = kD;
        error = 0;
        integral = 0;
        derivative = 0;
        pastError = 0;
        output = 0;
}

void computePID(PIDObject* pid, float error, float dT) {
        pid -> pastError = pid-> error;
        pid -> error = error;
        pid -> integral += error*dT;
        pid -> derivative = (pid->error - pid->pastError)/dT;

        pid -> output = (pid->kP*pid->error+pid->kI*pid->integral+pid->kD*pid->derivative);
}


        



