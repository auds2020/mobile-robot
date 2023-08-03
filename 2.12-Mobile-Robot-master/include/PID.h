//instantaneous velocity of each wheel in radians per second
extern float velFL;
extern float velBL;
extern float velFR;
extern float velBR;

//filtered velocity of each wheel in radians per second
extern float filtVelFL;
extern float filtVelBL;
extern float filtVelFR;
extern float filtVelBR;

//scaling factor for each new reading
//if alpha = 0, each new reading is not even considered
//if alpha = 1, each new reading is the only thing considered
//lower values of alpha smooth the filtered velocity more, but delay the signal
extern float alpha;

//sum errors for integral term
extern float sumErrorFL;
extern float sumErrorBL;
extern float sumErrorFR;
extern float sumErrorBR;

//desired velocity setpoints in rad/s
extern float desiredVelFL;
extern float desiredVelBL;
extern float desiredVelFR;
extern float desiredVelBR;

//voltage to send to the motors
extern float voltageFL;
extern float voltageBL;
extern float voltageFR;
extern float voltageBR;

//error reading
extern float errorFL;
extern float errorBL;
extern float errorFR;
extern float errorBR;

//PID Constants
extern float kp;
extern float ki;
extern float kd;

extern float lastRadFL;
extern float lastRadBL;
extern float lastRadFR;
extern float lastRadBR;

extern float dPhiFL;
extern float dPhiBL;
extern float dPhiFR;
extern float dPhiBR;


//function prototypes
void updateVelocity(float dt);
void getSetPointDriveTest(float angVel);
void getSetPointJoystick();
float runPID(float error,float last_error, float kp, float ki, float kd, float &sumError, float maxSumError, float loopTime);