void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void wirelessSetup(void);
typedef struct struct_message {
  uint16_t joyX;
  uint16_t joyY;
  bool rightPressed;
  bool downPressed;
  bool leftPressed;
  bool upPressed;
  bool selPressed;
} struct_message;

extern struct_message joyData;