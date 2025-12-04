#include <Arduino.h>
#include <WiFi.h>

#define ALWAYS_CALIBRATING CALIBRATION_LOOPS == -1

#define CALIB_OVERRIDE false
#if USING_CALIB_PIN && COMMUNICATION == COMM_SERIAL && PIN_CALIB == 0 && !CALIB_OVERRIDE
  #error "You can't set your calibration pin to 0 over usb. You can calibrate with the BOOT button when using bluetooth only. Set CalibOverride to true to override this."
#endif

ICommunication* comm;
int loops = 0;
void setup() {
  #if COMMUNICATION == COMM_SERIAL
    comm = new SerialCommunication();
  #endif  
  comm->start();

  setupInputs();

  #if USING_FORCE_FEEDBACK
    setupServoHaptics();  
  #endif

}

void loop() {
  if (comm->isOpen()){
    #if USING_CALIB_PIN
    bool calibButton = getButton(PIN_CALIB);
    if (calibButton)
      loops = 0;
    #else
    bool calibButton = false;
    #endif
    
    bool calibrate = false;
    if (loops < CALIBRATION_LOOPS || ALWAYS_CALIBRATING){
      calibrate = true;
      loops++;
    }
    
    int* fingerPos = getFingerPositions(calibrate, calibButton);
    
    comm->output(encode(fingerPos, calibButton));

    #if USING_FORCE_FEEDBACK
      char received[100];
      if (comm->readData(received)){
        int hapticLimits[5];
        
        if(String(received).length() >= 10) {
           decodeData(received, hapticLimits);
           writeServoHaptics(hapticLimits); 
        }
      }
    #endif
    delay(LOOP_TIME);
  }
}
