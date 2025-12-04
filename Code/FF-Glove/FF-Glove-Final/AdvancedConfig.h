#define LOOP_TIME 4 //How much time between data sends (ms), set to 0 for a good time :)
#define CALIBRATION_LOOPS -1//How many loops should be calibrated. Set to -1 to always be calibrated.

//Comm defines, no touchy
#define COMM_SERIAL 0   
#define COMM_BTSERIAL 1

//Encoding
#define ENCODING 1

//Automatically set ANALOG_MAX 
#define ANALOG_MAX 4095 

//Filtering and clamping analog inputs
#define CLAMP_ANALOG_MAP true //clamp the mapped analog values from 0 to ANALOG_MAX

// Enable and set min and max to match your sensor's expected raw value range
// This discards any spurious values outside of the useful range
#define CLAMP_FLEXION false  //clamp the raw flexion values
#define CLAMP_MIN 0  //the minimum value from the flexion sensors
#define CLAMP_MAX ANALOG_MAX  //the maximum value from the flexion sensors