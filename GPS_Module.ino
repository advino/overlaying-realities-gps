#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(3, 2); //Initialize the softwareserial port
Adafruit_GPS GPS(&mySerial);  //Create the GPS object

#define ledPin 12
#define heatPadFront 5
#define heatPad 7
#define switchPin 9
#define fixPin 11


String NMEA1; //variable for the first NMEA sentence
String NMEA2;//variable for the second NMEA setence
char c ; //reads the characters coming from GPS

float locationData[2][3] = {
  {41.823256, -71.406153, 0},
  {41.822696, -71.405448, 0}
};

float checkDistance(float myLat, float myLon) {
  for (int i = 0; i < 2; i ++) {
    float distance = locDistance(myLat, myLon, locationData[i][0], locationData[i][1]);
    if (distance <= 50) {
      if (locationData[i][2] == 0) {
        Serial.println("You are in a hot zone");
        Serial.print("Distance: "); Serial.print(distance);
        digitalWrite(ledPin, HIGH);
        digitalWrite(heatPad, HIGH);
        locationData[i][2] = 1;
      }
    } else {
      Serial.println("You are not in a hot zone");
      Serial.println("Distance: "); Serial.print(distance);
      //Check to see if I have stepped out of a current zone
      if (locationData[i][2] == 1) {
        digitalWrite(ledPin, LOW);
        digitalWrite(heatPad, LOW);
      }
      locationData[i][2] = 0;
    }
  }
}

float locDistance(float curLat, float curLon, float targetLat, float targetLon) {
  int r = 6371;
  float dLat = deg2Rad(targetLat - curLat);
  float dLon = deg2Rad(targetLon - curLon);
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(deg2Rad(curLat)) * cos(deg2Rad(targetLat)) *
            sin(dLon / 2) * sin(dLon / 2);

  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float d = r * c;
  return d * 1000;
}

float deg2Rad(float deg) {

  return deg * (M_PI / 180);
}


void setup() {
  Serial.begin(115200);
  GPS.begin(9600);
  GPS.sendCommand('$PGCMD,33,0*6D'); //turn off the antennae update
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //Set update rate to 1Hz
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //request the RMC and GGA Sentences
  pinMode(ledPin, OUTPUT);
  pinMode(heatPad, OUTPUT);
  pinMode(heatPadFront, OUTPUT);
  pinMode(switchPin, INPUT);
  pinMode(fixPin, OUTPUT);
  delay(1000);
}

void loop() {

  if (digitalRead(switchPin)) {
    readGPS(); 

    if (GPS.fix) {
    
      checkDistance(GPS.latitudeDegrees, GPS.longitudeDegrees);
      fixPos(2);
    }

    digitalWrite(ledPin, HIGH);
    digitalWrite(heatPadFront, HIGH);
    digitalWrite(heatPad, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
    digitalWrite(heatPadFront,LOW);
  }


}

void readGPS() {

  clearGPS();
  //loop until you have a good NMEA sentence
  while (!GPS.newNMEAreceived()) {
    c = GPS.read() ;

  }
  GPS.parse(GPS.lastNMEA()); //Parse the last good NMEA sentence
  NMEA1 = GPS.lastNMEA(); //Store this data to read later

  while (!GPS.newNMEAreceived()) {
    c = GPS.read() ;
  }
  GPS.parse(GPS.lastNMEA()); //Parse the last good NMEA sentence
  NMEA2 = GPS.lastNMEA(); //Store this data to read later

  Serial.println(NMEA1);
  Serial.println(NMEA2);
  Serial.println(' ');


}

void clearGPS() {
  //This clears old and corrupt data in GPS Sensor

  while (!GPS.newNMEAreceived()) {
    c = GPS.read() ;

  }
  GPS.parse(GPS.lastNMEA()); //Parse the last good NMEA sentence

  while (!GPS.newNMEAreceived()) {
    c = GPS.read() ;

  }
  GPS.parse(GPS.lastNMEA()); //Parse the last good NMEA sentence

  while (!GPS.newNMEAreceived()) {
    c = GPS.read() ;

  }
  GPS.parse(GPS.lastNMEA()); //Parse the last good NMEA sentence

}

void fixPos(uint8_t count) {
  for(int i = 0; i < count; i++) {
  digitalWrite(fixPin, HIGH);
  delay(50);
  digitalWrite(fixPin, LOW);
  delay(50);  
  }
  
  
}

