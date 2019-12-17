
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <FirebaseESP8266.h>

// WiFi Credentials
#ifndef STASSID
#define STASSID "AndroidAP"
#define STAPSK  "foxh6118"
#endif

// Firebase credentials
#ifndef FIREBASE_HOST
#define FIREBASE_HOST "garage-door-warning-system.firebaseio.com" 
#define FIREBASE_AUTH "gjUugc8Gz1UTi84SawRhHaVwfn5B33GzJqAdNj04" 
#endif

const char *ssid = STASSID;
const char *password = STAPSK;

FirebaseData firebaseData;

void setup(void) {
  
  // Begin establishing WiFi connection
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for WiFi to connect
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  // Display MAC Address
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  // Display IP Address
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Connect to Firebase DB
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  Serial.println("Connected to Firebase DB");
  
  // Set size and write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
}

// Main loop
void loop(void) {
  char sign, s;
  int GarageState = 0;
  double x = 0, y = 0, z = 0, t = 0, A = 0;
  boolean dataReady = false;
  String rxString, data;

  // Process incoming serial data until a full set of processed data is received from the MCU
  while (dataReady == false)
    if (Serial.available()) {
      char c = Serial.read(); // Read serial byte by byte
  
      if (c == ' ') { // Processed data is space delimited with value identifier in the front
        Serial.println(rxString); // Prints string with value to serial port out

        // Only the identifiers for the X,Y,Z angles, tempearture, Garage Tilt, and Garage State are processed
        // All other identifiers are ignored
        if(rxString.indexOf("X") >=0) {   // Process and store X angle
          sign = rxString.charAt(1);
          data = rxString.substring(2);
          Serial.print("X: ");
          Serial.println(sign + data);
          x = data.toDouble();
          if (sign == '-'){
            x *= -1;
          }
          Serial.println(x);
          Serial.println();
        }
        if(rxString.indexOf("Y") >=0) {   // Process and store Y angle
          sign = rxString.charAt(1);
          data = rxString.substring(2);
          Serial.print("Y: ");
          Serial.println(sign + data);
          y = data.toDouble();
          if (sign == '-'){
            y *= -1;
          }
          Serial.println(y);
          Serial.println();
        }
        if(rxString.indexOf("Z") >=0) {   // Process and store Z angle
          sign = rxString.charAt(1);
          data = rxString.substring(2);
          Serial.print("Z: ");
          Serial.println(sign + data);
          z = data.toDouble();
          if (sign == '-'){
            z *= -1;
          }
          Serial.println(z);
          Serial.println();
        }
        if(rxString.indexOf("T") >=0) {   // Process and store tempearture
          sign = rxString.charAt(1);
          data = rxString.substring(2);
          Serial.print("Temp: ");
          Serial.println(sign + data);
          t = data.toDouble();
          if (sign == '-'){
            t *= -1;
          }
          Serial.println(t);
          Serial.println();
        }
        if(rxString.indexOf("S") >=0) {   // Process and store Garage State
          Serial.print("Garage State: ");
          if (rxString == "S1"){
            Serial.println("Open");
            GarageState = 1;
          } else{
              Serial.println("Closed");
              GarageState = 0;
          }
          Serial.println(GarageState);
          Serial.println();
        }
        if(rxString.indexOf("A") >=0) {   // Process and store Garage Tilt
          sign = rxString.charAt(1);
          data = rxString.substring(2);
          Serial.print("Garage Angle: ");
          Serial.println(sign + data);
          A = data.toDouble();
          if (sign == '-'){
            A *= -1;
          }
          Serial.println(t);
          Serial.println();
          dataReady = true;   // Set dataReady flag since this is the final value in the dataset
        }
        rxString = ""; // Clears variable for new input
        data="";
      }
      else {
        rxString += c; // No "space" character so continue reading the string
      }
    }

  // Transmit the X, Y, Z angles, temperature, Garage State, Garage Tilt, and Timestamp to Firebase database
  Firebase.setDouble(firebaseData, "/ADXL362/XAng", x);
  Firebase.setDouble(firebaseData, "/ADXL362/YAng", y);
  Firebase.setDouble(firebaseData, "/ADXL362/ZAng", z);
  Firebase.setDouble(firebaseData, "/ADXL362/Temp", t);
  Firebase.setInt(firebaseData, "/ADXL362/GarageState", GarageState);
  Firebase.setDouble(firebaseData, "/ADXL362/GarageTilt", A);
  Firebase.setTimestamp(firebaseData, "/ADXL362/Time");

}
