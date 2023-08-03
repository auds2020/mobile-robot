//Diameter is 120mm converted to meters
#define WHEEL_RADIUS_M 0.06

extern long encFLCount;
extern long encBLCount;
extern long encFRCount;
extern long encBRCount;

extern float encFLRad;
extern float encBLRad;
extern float encFRRad;
extern float encBRRad;

void readEncoders();
void clearEncoders();
void encoderSetup();