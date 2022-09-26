/*
 * MIT License
 * 
 * Copyright (c) 2022 SM0SBL
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <SPI.h>
#include <WiFi101.h>
#include <Ethernet.h>

#include "arduino_secrets.h"
#include <Adafruit_SleepyDog.h>

#define KEYLED 6
#define KEYPIN 14
#define WIFILED 7

char* cred[2][2] = {{SSID1, PASS1}, {SSID2, PASS2}};
char keyctrl[1024];

int status = WL_IDLE_STATUS;

///////please enter your sensitive data in the Secret tab/arduino_secrets.h

unsigned int localPort = 6000;      // local port to listen on
unsigned long timeArrived, timeSent, timeOrigin;
unsigned long timeLeft;

char packetBuffer[2048]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back
int netid = -2;

WiFiServer server(localPort);
WiFiClient client;

int scanNetworks();

void(* resetFunc) (void) = 0;

void setup() {
  pinMode(KEYPIN, OUTPUT);
  pinMode(KEYLED, OUTPUT);
  pinMode(WIFILED, OUTPUT);
  digitalWrite(KEYPIN, HIGH);
  digitalWrite(KEYLED, LOW);
  digitalWrite(WIFILED, LOW);

  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  //  while (!Serial) {
  //    ; // wait for serial port to connect. Needed for debugging via USB port only
  //  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  status = WL_IDLE_STATUS;
  ConnectToWifi();

  Serial.println("Connected to wifi");
  printWiFiStatus();
  Serial.println("\nStarting connection to server...");

  server.begin();

  // Clear keyctrl time map
  for (int i = 0; i < 1024; i++) {
    keyctrl[i] = 0;
  }
  Serial.println("\nDone init...");

  // Start a watchdog to reset the board if
  //   e.g. it hangs or if it lose WiFi connection
  int countdownMS = Watchdog.enable(5000);
  int maxCountdownMS = Watchdog.enable();
}

void loop() {
  int i;
  char strSend[255];
  char str[255];
  char strArrived[100];
  int len;
  unsigned long timeToSend, now;
  static int keyctrlTime = millis();
  static unsigned long DownTimer = 0;
  int strpos = 0;
  // Reset watchdog to keep the board alive
  Watchdog.reset();
  while ( WiFi.status() != WL_CONNECTED ) {
    ; // if need to reconnect to wifi we need to reboot using the watchdog
  }

  if (!client.connected()) {
    client = server.available();
    if ( client ) {
      Serial.println("New client connected");
    }
  }

  now = millis();
  for (; keyctrlTime <= now; keyctrlTime++) {
    if ( keyctrl[(keyctrlTime % 1024)] == 'D' ) {
      digitalWrite(KEYPIN, LOW);
      digitalWrite(KEYLED, HIGH);
      // set keypin
      keyctrl[(keyctrlTime % 1024)] = 0;
    } else if ( keyctrl[(keyctrlTime % 1024)] == 'U' ) {
      // clear keypin
      digitalWrite(KEYPIN, HIGH);
      digitalWrite(KEYLED, LOW);
      keyctrl[(keyctrlTime % 1024)] = 0;
    }
  }

  if ( (DownTimer != 0) && ((now - DownTimer) > 10000) ) {
    digitalWrite(KEYPIN, HIGH);
    digitalWrite(KEYLED, LOW);
    DownTimer = 0;
  }
  if ( client && client.connected() ) {
    //if (packetSize) {
    timeArrived = now;
    len = 0;
    while (client.available()) {
      packetBuffer[len++] = client.read();
    }
    packetBuffer[len] = 0;
    if ( len > 0 ) {
      strpos = 3;
      sscanf((packetBuffer + 3), "%lu %lu", &timeOrigin, &timeToSend);
      if ( packetBuffer[0] == 'K' ) { //Key command
        if ( packetBuffer[1] == 'D' ) {
          DownTimer = now;
          keyctrl[(timeToSend % 1024)] = 'D';
          sprintf(str, "KD ");
        } else if ( packetBuffer[1] == 'U' ) {
          DownTimer = 0;
          keyctrl[(timeToSend % 1024)] = 'U';
          sprintf(str, "KU ");
        } else {
          // send a reply, to the IP address and port that sent us the packet we received
          client.print("Unknown subcommand received: ");
          client.print(packetBuffer[1]);
          client.print("\n\r");
        }
      } else if ( packetBuffer[0] == 'P' ) { //Ping command
        sprintf(str, "PP ");
      } else {
        // send a reply, to the IP address and port that sent us the packet we received
        client.print("Unknown command received: ");
        client.print(packetBuffer[0]);
        client.print("\n\r");
      }
      timeSent = millis();
      strpos += sprintf(str+strpos, "%lu %lu %lu", timeOrigin, timeSent, timeToSend);
      client.print(str);
      Serial.print("Packet received: [");
      Serial.print(packetBuffer);
      Serial.println("]");
      Serial.print("TimeOrigin=");
      Serial.print(timeOrigin);
      Serial.print(" TimeSent=");
      Serial.print(timeSent);
      Serial.print(" timeToSend=");
      Serial.println(timeToSend);
      sprintf(strSend, "uptime: %lu:%02lu:%02lu:%03lus", now / (60 * 60 * 1000), (now / (60 * 1000)) % 60, (now / 1000) % 60, now % 1000);
      Serial.println(strSend);
    }
  } else if ( client ) {
    client.stop();
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void ConnectToWifi() {

  WiFi.disconnect();
  delay(1000);
  netid = scanNetworks();
  delay(1000);

  // attempt to connect to WiFi network:
  Serial.print("netid=");
  Serial.println(netid);

  status = WL_IDLE_STATUS;
  while ( (status != WL_CONNECTED) && (netid != -1) ) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(cred[netid][0]);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(cred[netid][0], cred[netid][1]);
    // wait 10 seconds for connection:
    Serial.println("Waiting for 10 seconds");
    delay(10000);
  }
  digitalWrite(WIFILED, HIGH);

}

int scanNetworks() {
  // scan for nearby networks:
  Serial.println("** Scan Networks **");
  byte numSsid = WiFi.scanNetworks();
  int i;
  // print the list of networks seen:
  Serial.print("SSID List:");
  Serial.println(numSsid);
  // print the network number and name for each network found:
  for (int thisNet = 0; thisNet < numSsid; thisNet++) {
    Serial.print(thisNet);
    Serial.print(") Network: ");
    Serial.println(WiFi.SSID(thisNet));
  }
  for (int thisNet = 0; thisNet < numSsid; thisNet++) {
    for (i = 0; i < 2; i++) {
      if ( 0 == strcmp(WiFi.SSID(thisNet), cred[i][0]) ) { //, sizeof(cred[i][0])) ) {
        Serial.print("WiFi network found: ");
        Serial.println(cred[i][0]);
        return i;
      } else {
        Serial.println("No match found!");
      }
    }
  }
  return -1;
}
