#include <sbus.h>
#include <HardwareSerial.h>

#define FAILSAFE_TIMEOUT 1000  // allowable time between connection pings before a failsafe is triggered

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2, 19, 23, true);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial2, 19, 23, true);
/* SBUS data */
bfs::SbusData data;
int channels[data.NUM_CH];

void setup() {
  Serial.begin(115200);
  sbus_rx.Begin();
  Serial.println("Starting...");
}

void loop() {
  if (sbus_rx.Read()) {
    /* Grab the received data */
    data = sbus_rx.data();  // raw data from the receiver
    /* Map the received data */
    for (int i = 0; i < data.NUM_CH; i++) {
      channels[i] = map(data.ch[i], 172, 1812, -1000, 1000);
    }
  }

  

  for (int i = 0; i < data.NUM_CH; i++) {
    Serial.print("CH");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(channels[i]);
    Serial.print(" ");
  }
  Serial.println();
  
}
