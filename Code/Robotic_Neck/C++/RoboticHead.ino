

class RoboticHead(Servo Yaw, Servo Left, Servo Right) {
  public:
    void begin();
    void update();
    int pitch;
    int roll;
    int yaw;
    

    int getPitch();
    int getRoll();
    int getYaw();

    void approachPitch(int targetPitch);
    void approachRoll(int targetRoll);
    void approachYaw(int targetYaw);
    
};