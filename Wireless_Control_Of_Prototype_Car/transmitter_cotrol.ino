#include<SoftwareSerial.h>
SoftwareSerial telemetrySerial(5,6);

struct TelemetryData { // Define a struct to hold the telemetry data
  char a;
};

void setup() {

  Serial.begin(9600);
  telemetrySerial.begin(9600);

}

void loop() {
  char ax = Serial.read();
  Serial.println(ax);
  TelemetryData data;
  data.a = ax;

  byte dataBytes[sizeof(data)];
  memcpy(dataBytes, &data, sizeof(data));
      

  // Transmit the encapsulated telemetry data
  telemetrySerial.write(dataBytes, sizeof(dataBytes));
  delay(100);


}
