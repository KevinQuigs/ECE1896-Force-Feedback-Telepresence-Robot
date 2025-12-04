
#if ENCODING 
char* encode(int* flexion, bool calib){
  static char stringToEncode[75];
  sprintf(stringToEncode, "%d,%d,%d,%d,%d,%d\n", 
    flexion[0], flexion[1], flexion[2], flexion[3], flexion[4],
    calib ? 1 : 0
  );
  return stringToEncode;
}

//legacy decoding
void decodeData(char* stringToDecode, int* hapticLimits){
  hapticLimits[0] = getArgument(stringToDecode, 'A'); //thumb
  hapticLimits[1] = getArgument(stringToDecode, 'B'); //index
  hapticLimits[2] = getArgument(stringToDecode, 'C'); //middle
  hapticLimits[3] = getArgument(stringToDecode, 'D'); //ring
  hapticLimits[4] = getArgument(stringToDecode, 'E'); //pinky
  //Serial.println("Haptic: "+ (String)hapticLimits[0] + " " + (String)hapticLimits[1] + " " + (String)hapticLimits[2] + " " + (String)hapticLimits[3] + " " + (String)hapticLimits[4] + " ");
}

int getArgument(char* stringToDecode, char command){
  char* start = strchr(stringToDecode, command);
  if (start == NULL)
    return -1;
  else
    return atoi(start + 1);
}

#endif