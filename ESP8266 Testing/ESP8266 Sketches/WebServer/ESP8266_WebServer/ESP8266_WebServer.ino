/*
   Copyright (c) 2015, Majenko Technologies
   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

 * * Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

 * * Redistributions in binary form must reproduce the above copyright notice, this
     list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

 * * Neither the name of Majenko Technologies nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <FirebaseESP8266.h>

#ifndef STASSID
#define STASSID "NotaVirus"
#define STAPSK  "Buffalo1226"
#endif

#ifndef FIREBASE_HOST
#define FIREBASE_HOST "garage-door-warning-system.firebaseio.com" 
#define FIREBASE_AUTH "gjUugc8Gz1UTi84SawRhHaVwfn5B33GzJqAdNj04" 
#endif

const char *ssid = STASSID;
const char *password = STAPSK;

FirebaseData firebaseData;

ESP8266WebServer server(80);

const int led = 14;

void handleRoot() {
  char temp[400];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;
  double x = 0, y = 0, z = 0, t = 0;
  boolean dataReady = false;
  String rxString, data, sign;

  while (dataReady == false)
    if (Serial.available()) {
      char c = Serial.read();
  
      if (c == ' ') {
        Serial.println(rxString); //prints string to serial port out
       
        if(rxString.indexOf("X") >=0) {
          sign = rxString.substring(1,1);
          data = rxString.substring(2);
          Serial.print("X: ");
          Serial.println(sign + data);
          x = data.toDouble();
          if (sign == "-"){
            x *= -1;
          }
          Serial.println(x);
          Serial.println();
        }
        if(rxString.indexOf("Y") >=0) {
          sign = rxString.substring(1,1);
          data = rxString.substring(2);
          Serial.print("Y: ");
          Serial.println(sign + data);
          y = data.toDouble();
          if (sign == "-"){
            y *= -1;
          }
          Serial.println(y);
          Serial.println();
        }
        if(rxString.indexOf("Z") >=0) {
          sign = rxString.substring(1,1);
          data = rxString.substring(2);
          Serial.print("Z: ");
          Serial.println(sign + data);
          z = data.toDouble();
          if (sign == "-"){
            z *= -1;
          }
          Serial.println(z);
          Serial.println();
        }
        if(rxString.indexOf("T") >=0) {
          sign = rxString.substring(1,1);
          data = rxString.substring(2);
          Serial.print("Temp: ");
          Serial.println(sign + data);
          t = data.toDouble();
          if (sign == "-"){
            t *= -1;
          }
          Serial.println(sign + t);
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

  snprintf(temp, 400,

           "<html>\
  <head>\
    <meta http-equiv='refresh' content='5'/>\
    <title>ESP8266 Demo</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>\
    <h1>Hello from ESP8266!</h1>\
    <p>Uptime: %02d:%02d:%02d</p>\
    <p>ADXL362 Data: x:%04lf, y:%04lf, z:%04lf, temp:%04lf</p>\
  </body>\
</html>",

           hr, min % 60, sec % 60, x, y, z, t
          );
  server.send(200, "text/html", temp);
}

void handleNotFound() {
  digitalWrite(led, 0);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, "text/plain", message);
  digitalWrite(led, 0);
}

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
  
  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);
  server.on("/inline", []() {
    server.send(200, "text/plain", "this works as well");
  });
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
}

void loop(void) {
  server.handleClient();
  MDNS.update();
}
