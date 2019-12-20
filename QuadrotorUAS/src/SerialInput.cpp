#include "SerialInput.h"

char* inputSerial(char * input) {

  if(Serial.available()) {
    for(int i=0; i<9; ++i) {
      char c = Serial.read();
      if(c == '\n') break;
      *input += c;
    }
  }
  return input;
}


int processSerial(char* input) {

       //--------------------------------------|bluetooth processing|------------------------------------------------------------//

    switch(input[0]) {
      case 'S':             // SET
        input[1] = 'F';
    }


    //----------------------------------------------|end bluetooth processing|----------------------------------------------//
    return 1;
}