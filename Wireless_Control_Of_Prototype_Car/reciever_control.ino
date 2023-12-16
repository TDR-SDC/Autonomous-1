#include <SoftwareSerial.h>

SoftwareSerial telemetrySerial(10,11); // Create a software serial object for telemetry reception

struct TelemetryData { // Define a struct to hold the telemetry data
  String a;
};

const int stepPin = 13;
const int dirPin = 12;
const int enPin = 8;
int motor1pin1= 3;
int motor1pin2= 4;
int motor2pin1= 6;
int motor2pin2= 7;
int en1 = 9;
int en2 = 5; 

void setup() {
  
  pinMode(stepPin,OUTPUT);
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);


  Serial.begin(9600); // Initialize the serial port for debugging
  telemetrySerial.begin(9600); // Initialize the telemetry serial por


}



void loop() {
  TelemetryData data; // Create a TelemetryData object to hold the received data
  if (telemetrySerial.available() >= sizeof(TelemetryData)) { // Check if a full telemetry packet has been received
    byte dataBytes[sizeof(TelemetryData)];
    telemetrySerial.readBytes(dataBytes, sizeof(dataBytes)); // Read the telemetry packet into a byte array

    
    memcpy(&data, dataBytes, sizeof(data)); // Decapsulate the received data into the TelemetryData object
    
    Serial.print(data.a);
  }



  
    String control;
//  if(Serial.available()>0){
    control = data.a;
  
    if(control=="a"){
        digitalWrite(dirPin, 1);
        for(int i=0;i<20;i++){
          digitalWrite(stepPin, HIGH);
          delay(1);
          digitalWrite(stepPin, LOW);
          delay(1);
        }
    }
    if(control=="d"){
//        direction = 0;
        digitalWrite(dirPin, 0);
        digitalWrite(motor2pin1,0);
        digitalWrite(motor2pin2,0);

        for(int i=0;i<20;i++){
          digitalWrite(stepPin, HIGH);
          delay(1);
          digitalWrite(stepPin, LOW);
          delay(1);
        }
    } 
    if(control=="w"){
      digitalWrite(motor1pin1,HIGH);
      digitalWrite(motor1pin2,LOW);
      digitalWrite(motor2pin1,HIGH);
      digitalWrite(motor2pin2,LOW);
      for(int i=0;i<10;i++){
        analogWrite(en1,255);
        analogWrite(en2,255);
      }
    }
    if(control=="s"){
      digitalWrite(motor1pin1,LOW);
      digitalWrite(motor1pin2,HIGH);
      digitalWrite(motor2pin1,LOW);
      digitalWrite(motor2pin2,HIGH);
      for(int i=0;i<10;i++){
        analogWrite(en1,0);
        analogWrite(en2,0);
      }
    }
  delay(100);
}    
