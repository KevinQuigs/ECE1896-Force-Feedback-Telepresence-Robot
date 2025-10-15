#include <Arduino.h>
#include <ESP32Servo.h>

//Pin definitions for the servos
#define PIN_LEFT_SERVO 12
#define PIN_RIGHT_SERVO 13
#define PIN_YAW_SERVO 14

Servo LEFT_MOTOR;
Servo RIGHT_MOTOR;
Servo YAW_MOTOR;

float pitch_target = 1.5708;
float roll_target = 1.5708;
float yaw_target = 1.5708;
String serialDataIn = "";

  //CommunicationI* comm;
void setup() {
  /*#if COMMUNICATION == COMM_SERIAL
    comm = new SerialCommunication();
  #elif COMMUNICATION == COMM_BTSERIAL
    comm = new BTSerialCommunication();
  #endif  
  comm->start();
  */
  Serial.begin(115200);
  
  // Attach servo to pin, set min and max pulse width (microseconds)
  LEFT_MOTOR.attach(PIN_LEFT_SERVO, 500, 2500); 
  RIGHT_MOTOR.attach(PIN_RIGHT_SERVO, 500, 2500); 
  YAW_MOTOR.attach(PIN_YAW_SERVO, 500, 2500); 
}



void loop() {
  //if (comm->isOpen()){
    //Read serial data and split into respective vars
    
    //if (comm->readData(serialDataIn)){
    if(Serial.available() > 0){
      serialDataIn = Serial.readStringUntil('\n');
      serialDataIn.trim();
      int index1 = serialDataIn.indexOf(',');
      pitch_target = (serialDataIn.substring(0, index1).toFloat());
      
      int index2 = serialDataIn.indexOf(',', index1 + 1); 
      roll_target = (serialDataIn.substring(index1 + 1, index2).toFloat());

      yaw_target = (serialDataIn.substring(index2 + 1).toFloat());



      float pitch_ang = calculateAngleForPitch(pitch_target);
      float roll_disp = solveForDisplacementRoll(radToDeg(roll_target));
      float left_motor_ang = 90;
      float right_motor_ang = 90;

      //if roll_disp is > 0 then left motor will be set to a lower angle (lowers actuator)
      if (roll_disp > 0){
        left_motor_ang = pitch_ang - roll_disp;
        right_motor_ang = pitch_ang + roll_disp;
        
        if(left_motor_ang < 0){
          float new_disp = roll_disp + left_motor_ang;
          left_motor_ang = pitch_ang - new_disp;
          right_motor_ang = pitch_ang + new_disp;
        }else if(right_motor_ang > 180){
          float new_disp = roll_disp - 180 + right_motor_ang;
          right_motor_ang = pitch_ang + new_disp;
          left_motor_ang = pitch_ang - new_disp;
        }

      }else{ //roll_disp is negative
        left_motor_ang = pitch_ang - roll_disp;
        right_motor_ang = pitch_ang + roll_disp;

        if(right_motor_ang < 0){
          float new_disp = roll_disp - right_motor_ang;
          right_motor_ang = pitch_ang + new_disp;
          left_motor_ang = pitch_ang - new_disp;
        }else if(left_motor_ang > 180){
          float new_disp = roll_disp - 180 + left_motor_ang;
          left_motor_ang = pitch_ang - new_disp;
          right_motor_ang = pitch_ang + new_disp;
        }
      }

      RIGHT_MOTOR.write(calculateRightMotorAngle(right_motor_ang));
      LEFT_MOTOR.write((int)left_motor_ang);

      //setMotorAngles(pitch_target, roll_target, LEFT_MOTOR, RIGHT_MOTOR);

      //LEFT_MOTOR.write(calculateAngleForPitch(serialDataIn.toFloat()));
      //RIGHT_MOTOR.write(calculateRightMotorAngle(LEFT_MOTOR.read()));
      //RIGHT_MOTOR.write(LEFT_MOTOR.read()+ 5);
      YAW_MOTOR.write(calculateAngleForYaw(yaw_target));
    }

    
  //}

    

}

//Takes in the desired angle (from VR headset over serial comm), and current angle (from left motor)
int calculateAngleForPitch(float desired_angle_rads){
  float des_ang = desired_angle_rads;
  if (desired_angle_rads < -0.279){
    des_ang = -0.279;
  }else if (desired_angle_rads > 0.453){
    des_ang = 0.453;
  }
  return solveForX(radToDeg(desired_angle_rads));
}

//Takes in the desired angle in degrees and uses line best fit to calculate the servo angles
float solveForX(float des_ang) {
  float x = 0;

  if(des_ang <= -14.6){
    x = (des_ang + 16.133)/0.07;
  }else if(-14.6 < des_ang <= -11.44){
    x = (des_ang + 16.804)/0.1369;
  //}else if(-13.32 < des_ang <= -12){
  //  x = (des_ang - 4.3767)/1.1874;
  }else if(-11.44 < des_ang <= 13.68){
    x = (des_ang + 24.682)/0.3125;
  }else if(13.68 < des_ang <= 19.62){
    x = (des_ang + 21.957)/0.297;
  }else if(19.62 < des_ang <= 24.04){
    x = (des_ang + 11.227)/0.221;
  }else{
    x = (des_ang - 8.6733)/0.096;
  }

  return x; // return approximate result
}


//Since the right and left motors spin in opposing directions this function converts an angle (using the direction of the left motor) to an angle usable by the right motor
int calculateRightMotorAngle(int leftMotorAngleRef){
  int angle = 185 - leftMotorAngleRef;
  if (angle > 180){
    angle = 180;
  }
  return angle;
}

//Calculates the angle for the yaw motor
//Applies designated range (1-179)
int calculateAngleForYaw(float desired_angle_rads){
  int des_ang = radToDeg(desired_angle_rads);

  if(des_ang < 1){
    return 1;
  }else if(des_ang > 179){
    return 179;
  }else{
    return des_ang;
  }
}

float solveForDisplacementRoll(float desired_angle_rads){
  float disp = 0;

  if (desired_angle_rads >= 23.42){
    disp = (desired_angle_rads + 5.6937)/1.2678;
    if (disp < -180){ disp = -180;}
  }else if (23.42 > desired_angle_rads >= 19.96){
    disp = (desired_angle_rads + 1.353)/1.1716;
  }else if (19.96 > desired_angle_rads >= -16.61){
    disp = (desired_angle_rads - 0.9351)/(-0.2079);
  }else if (-16.61 > desired_angle_rads >= -20.78){
    disp = (desired_angle_rads + 4.4055)/(-0.1426);
  }else if (-20.78 > desired_angle_rads >= -23.25){
    disp = (desired_angle_rads + 1.9336)/(0.8106);
  }else{
    disp = (desired_angle_rads + 3.6836)/ 0.8152;
    if (disp > 180){ disp = 180;}
  }

  return disp / 2;
}

//converts radian value to degrees
float radToDeg(float rad){
  return (rad * 180/PI);
}

int setMotorAngles(float des_ang_pitch, float des_ang_roll, Servo &MOTOR_LEFT, Servo &MOTOR_RIGHT){
  float pitch_ang = calculateAngleForPitch(des_ang_pitch);
  float roll_disp = solveForDisplacementRoll(radToDeg(des_ang_roll));
  float left_motor_ang = 90;
  float right_motor_ang = 90;

  //if roll_disp is > 0 then left motor will be set to a lower angle (lowers actuator)
  if (roll_disp > 0){
    left_motor_ang = pitch_ang - roll_disp;
    right_motor_ang = pitch_ang + roll_disp;
    
    if(left_motor_ang < 0){
      float new_disp = roll_disp + left_motor_ang;
      left_motor_ang = pitch_ang - new_disp;
      right_motor_ang = pitch_ang + new_disp;
    }else if(right_motor_ang > 180){
      float new_disp = roll_disp - 180 + right_motor_ang;
      right_motor_ang = pitch_ang + new_disp;
      left_motor_ang = pitch_ang - new_disp;
    }

  }else{ //roll_disp is negative
    left_motor_ang = pitch_ang - roll_disp;
    right_motor_ang = pitch_ang + roll_disp;

    if(right_motor_ang < 0){
      float new_disp = roll_disp - right_motor_ang;
      right_motor_ang = pitch_ang + new_disp;
      left_motor_ang = pitch_ang - new_disp;
    }else if(left_motor_ang > 180){
      float new_disp = roll_disp - 180 + left_motor_ang;
      left_motor_ang = pitch_ang - new_disp;
      right_motor_ang = pitch_ang + new_disp;
    }
  }

  MOTOR_RIGHT.write(calculateRightMotorAngle(right_motor_ang));
  RIGHT_MOTOR.write((int)left_motor_ang);
}

