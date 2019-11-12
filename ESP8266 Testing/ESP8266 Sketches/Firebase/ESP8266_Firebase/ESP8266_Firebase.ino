#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <FirebaseESP8266.h>

#ifndef STASSID
#define STASSID "NotaVirus"
#define STAPSK  "Buffalo1226"
#endif

#ifndef FIREBASE_HOST
#define FIREBASE_HOST "androidtest-2aab5.firebaseio.com" 
#define FIREBASE_AUTH "AIzaSyAh-bNTSRzxib8yWfCd8QyQ7fxIywo1DXE" 
#endif

const char *ssid = STASSID;
const char *password = STAPSK;

FirebaseData firebaseData;
FirebaseJson jsonBuffer;

const int led = 14;

void setup(void) {
  // Establish WiFi connection
  pinMode(led, OUTPUT);
  digitalWrite(led, 0);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  digitalWrite(led, 1);
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Connect to Firebase DB
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  Serial.println("Connected to Firebase DB");

  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
}

void loop(void) {
  char temp[400], sign;
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;
  double x = 0, y = 0, z = 0, t = 0;
  boolean dataReady = false;
  String jsonStr, rxString, data;

  while (dataReady == false)
    if (Serial.available()) {
      char c = Serial.read();
  
      if (c == ' ') {
        Serial.println(rxString); //prints string to serial port out
       
        if(rxString.indexOf("X") >=0) {
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
        if(rxString.indexOf("Y") >=0) {
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
        if(rxString.indexOf("Z") >=0) {
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
        if(rxString.indexOf("T") >=0) {
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
          dataReady = true;
        }
        rxString = ""; //clears variable for new input
        data="";
      }
      else {
        rxString += c; // Continue reading the string
      }
    }

  //String fireX = String(x) + String("%");
  //String fireY = String(y) + String("%");
  //String fireZ = String(z) + String("%");
  //String fireTemp = String(t) + String("%");
  Firebase.pushDouble(firebaseData, "ADXL362/X", x);
  Firebase.pushDouble(firebaseData, "ADXL362/Y", y);
  Firebase.pushDouble(firebaseData, "ADXL362/Z", z);
  Firebase.pushDouble(firebaseData, "ADXL362/T", t);
  Firebase.pushTimestamp(firebaseData, "ADXL362/Time");

}
