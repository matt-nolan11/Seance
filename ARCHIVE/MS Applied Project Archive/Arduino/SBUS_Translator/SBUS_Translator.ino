#include "sbus.h"
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial1, true);
/* SBUS data struct*/
bfs::SbusData data;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

#define inputRange 1000 // range of inputs that can be received from the PC
unsigned long currentTime; // tracks how many milliseconds have passed since the program started
int16_t wheels[4]; // wheel input values received from python, 

boolean newData = false;

void inputMap(int16_t channels[4]) {
   for (int i=0; i<4; i++) {
    data.ch[i] = map(channels[i], -inputRange, inputRange, 172, 1812); // fill the channel attribute of the data struct with mapped channel values
  }
}

void sbusWrite(int16_t channels[4]) {
  static unsigned long previousTime;
  currentTime = millis();

  if (currentTime-previousTime>=10) {
    inputMap(channels); // map inputs to the range expected by the radio TX
    sbus_tx.data(data); // fill the SBUS data object with the configured data struct
    sbus_tx.Write(); // write the SBUS data to the transmitter
    previousTime = currentTime;
  }
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");
    wheels[0] = atoi(strtokIndx);     // convert this section of the string to an integer

    strtokIndx = strtok(NULL, ",");
    wheels[1] = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    wheels[2] = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    wheels[3] = atoi(strtokIndx); 
}

void setup() {
  Serial.begin(115200);
  sbus_tx.Begin();
  
}

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    // because strtok() used in parseData() replaces the commas with \0
    parseData();
    newData = false;
   }
   
  sbusWrite(wheels);
}
