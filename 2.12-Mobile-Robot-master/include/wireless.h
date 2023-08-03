void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void wirelessSetup(void);
typedef struct joy_message {
  uint16_t joyX;
  uint16_t joyY;
  bool rightPressed;
  bool downPressed;
  bool leftPressed;
  bool upPressed;
  bool selPressed;
} joy_message;

extern joy_message joyData;

typedef struct odometry_message {
  unsigned long millis;
  float pathDistance;
  float x;
  float y;
  float theta;
  float velL;
  float velR;
} odometry_message;

extern odometry_message odom_data;

bool sendOdometry();