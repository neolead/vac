#include <ESP8266WiFi.h>        // Wifi library for ESP
#include <BlynkSimpleEsp8266.h> // Blynk library
#include <ArduinoOTA.h>         // Library for over the air updates
//#include <IRremote.h>         // IR remote
#include <IRremoteESP8266.h>    // IR remote lib for esp8266
#include <IRsend.h>             // IR sending
#include <IRrecv.h>
#include <math.h>

// Wifi, OTA, blynk credentials
const char* ssid = "WIFINAME";                              // SSID
const char* password = "WIFIPASS";                   // Password
char auth[] = "TOKEN";                               // Token for Esp8266
const char* OTAName = "update";                       // OTA name
const char* OTAPassword = "update";                    // OTA Password

// Prepare IR LED to be used
#define IR_LED 4                // ESP8266 GPIO pin to use for blaster (4 is D2 on nodeMCU devBoard)
#define IR_RECV 5               // ESP8266 GPIO pin to use for reciever (5 is D1 on nodeMCU devBoard)
IRrecv irrecv(IR_RECV);
IRsend irsend(IR_LED);          // Set the GPIO to send IR signals
int ub = 0;
int ubp = 0;
int ubh = 0;
int ubc = 0;
int ubm = 0;
int ts = 0;
int timergo = 0;

// Pins for blynk
#define VPIN_clean V1           // Switch AUTO clean ON or OFF
#define VPIN_home V2            // Homing signal
#define VPIN_play V3            // Play/ pause
#define VPIN_dpad V4            // D-pad
#define VPIN_room V5            // Room
#define VPIN_max V6             // Max
#define VPIN_wall V7            // Wall
#define VPIN_timerONOFF V8      // Timer ON/ OFF
#define VPIN_timer V9           // Timer handle
#define VPIN_spot V10           // Spot

#define VPIN_everything V21     // Tag: everything can be used to group devices and give one command to switch off/ on everything connected to blynk app
#define VPIN_terminal V25       // Terminal monitoring through Blynk app

#define VPIN_connected V30      // Connection

// Define variables
boolean LampState = 0;
boolean SwitchReset = true;     // Flag indicating that the hardware button has been released
int khz = 38;                   // 38kHz carrier frequency
int Timer_status = 0;

// Start timer
BlynkTimer timer;

// Terminal for Blynk
WidgetTerminal terminal(VPIN_terminal);

// Define all the IR codes:

uint16_t ir_auto[54] = {3042, 1010, 690, 1518, 1506, 702, 686, 1518, 686, 1522, 1502, 706, 1510, 698, 1502, 706, 1506, 714, 3042, 1010, 682, 1522, 1506, 702,686, 1522, 658, 1550, 1478, 730, 1482, 726,1474, 734, 1478, 742, 3006, 1042, 658, 1550, 1478, 730, 658, 1546, 654, 1550, 1478, 730, 1482, 726, 1474, 734, 1478, 1000};
uint16_t ir_home[54] = {3018, 1034, 658, 1550, 1474, 734, 654, 1554, 1474, 734, 654, 1554, 658, 1546, 1482, 754, 1458, 5482, 3014, 1038, 662, 1546, 1482, 754, 634, 1542, 1482, 754, 634, 1546, 658, 1550, 1474, 758, 1458, 5482, 3010, 1042, 662, 1546, 1478, 758, 630, 1546, 1482, 754, 634, 1546, 658, 1550, 1474, 758, 1454, 1000};  // UNKNOWN E8BEB62D
uint16_t ir_play[18] = {3038, 1018, 682, 1526, 1502, 706, 1506, 702, 1510, 702, 1510, 698, 690, 1518, 1510, 698, 686, 1000};  // UNKNOWN C1C301C7
uint16_t ir_pause[18] = {3038, 1018, 682, 1526, 1502, 706, 1506, 702, 1510, 702, 1510, 698, 690, 1518, 1510, 698, 686, 1000};  // UNKNOWN 25638DD6

// D-pad directionality
uint16_t ir_left[17] = {2980,1020, 680,1470, 1530,720, 630,1520, 680,1520, 680,1470, 1480,720, 680,1520, 1480};
uint16_t ir_down[17] = {3030,970, 730,1470, 1530,620, 730,1470, 730,1470, 730,1470, 730,1420, 1580,620, 1530};
uint16_t ir_right[17] = {2980,1020, 730,1470, 1480,670, 730,1470, 730,1470, 730,1420, 1480,720, 1480,720, 1480};
uint16_t ir_up[17] = {3030,1020, 630,1520, 1480,720, 680,1470, 680,1520, 680,1520, 680,1470, 680,1520, 1480}; 

// Single room
uint16_t ir_room[83] = {3000,3000, 500,500, 500,1500, 500,1500, 500,500, 500,1500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,1500, 500,1500, 500,500, 500,1500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,1500, 500,500, 500,500, 500,1500, 500,1500, 500,1500, 500,500, 500,500, 500};  // UNKNOWN 1A8F2EB8

// Max cleaning
uint16_t ir_max[54] = {3042, 1014, 686, 1522, 1502, 706, 682, 1526, 686, 1518, 1510, 702, 686, 1518, 1506, 702, 686, 6282, 3038, 1014, 686, 1522, 1506, 702,686, 1522, 690, 1514, 1510, 702, 686, 1518,1510, 698, 690, 6278, 3038, 1014, 690, 1518, 1506, 702, 686, 1522, 682, 1522, 1502, 706, 682, 1526, 1502, 706, 690, 1000};  // UNKNOWN A2B091CB

// Wall cleaning
uint16_t ir_wall[83] = {3000,3000, 500,500, 500,1500, 500,1500, 500,500, 500,1500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,1500, 500,500, 500,500, 500,500, 500,1500, 500,1500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,500, 500,1500, 500,500, 500,500, 500,500, 500,1500, 500,500, 500,1500, 500,1500, 500};  // UNKNOWN FFB2DFB1

// Spot cleaning
uint16_t ir_spot[54] = {3014, 1038, 654, 1554, 1482, 726, 662, 1546, 658, 1550, 1474, 734, 1478, 734, 654, 1550, 1478, 742, 3014, 1038, 662, 1546, 1482, 726,662, 1546, 654, 1554, 1474, 734, 1478, 730, 658, 1550, 1474, 746, 3010, 1042, 662, 1542, 1482, 730, 658, 1546, 658, 1550, 1474, 734, 1478, 730, 658, 1550, 1478, 1000};  // UNKNOWN FFB2DFB1

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting vacuum cleaner controller");

  // Wifi connection and services
  startWiFi();                      // Connect to WIFI
  startOTA();                       // Start OTA servers

  // Check internet connection every 5 minutes
  timer.setInterval(300000L, CheckConnection); // check if still connected every 5 minutes  

  // Start configuration
  Blynk.config(auth);               // Connect to Blynk
  Blynk.connect();

  // WDT handling
  ESP.wdtDisable();
  ESP.wdtEnable(WDTO_8S);
  timer.setInterval(100000, WDT);

  irsend.begin();
  irrecv.enableIRIn(); // Start the receiver
}

void CheckConnection(){    // check every 5 minutes if connected to Blynk server
  if(!Blynk.connected()){
    Serial.println("Not connected to Blynk server"); 
    Blynk.connect();  // try to connect to server with default timeout
  }
  else{
    Serial.println("Connected to Blynk server");  
    terminal.println("Connected to Blynk server");  
    Blynk.virtualWrite(VPIN_connected, 1);   // Denote that ESP is connected to Blynk server
  }
}

void loop() {
  if(Blynk.connected()){
    Blynk.run();
  }
  ArduinoOTA.handle();
  timer.run();
}

BLYNK_CONNECTED() {
    Blynk.syncAll();
}

// Start cleaning in auto mode
BLYNK_WRITE(VPIN_clean){
  
  int Clean_button = param.asInt();
  if (Clean_button == 1 && ubc == 0){
    ubc = 1;
    irsend.sendRaw(ir_play, 18, khz);   // Send Auto Clean signal on press
    delay(300);
    irsend.sendRaw(ir_auto, 54, khz);   // Send Auto Clean signal on press
    Blynk.virtualWrite(VPIN_play, 1);   // Denote that vacuum cleaner is in a cleaning mode
    Blynk.virtualWrite(VPIN_clean, 1);   // Denote that vacuum cleaner is in a cleaning mode
    delay(300);
    terminal.println("Eufy will clean the rooms");
    Serial.println("Eufy will clean the rooms!");
    
  }
  if (Clean_button == 0 && ubc == 1){
    ubc = 0;
    irsend.sendRaw(ir_play, 18, khz);   // Send Auto Clean signal on press
    delay(300);
    Blynk.virtualWrite(VPIN_play, 0);   // Denote that vacuum cleaner is in a cleaning mode
    Blynk.virtualWrite(VPIN_clean, 0);   // Denote that vacuum cleaner is in a cleaning mode
    delay(300);
    terminal.println("Eufy will clean the rooms stop");
    Serial.println("Eufy will clean the rooms! stop");
  }
}

// Send Vacuum cleaner home
BLYNK_WRITE(VPIN_home){
  int Clean_button = param.asInt();

  if (Clean_button == 1 && ubh ==0){
    ubh = 1;
    irsend.sendRaw(ir_play, 18, khz);   // Send Play signal on press
    delay(300);
    irsend.sendRaw(ir_home, 54, khz);   // Send Home signal on press
    Blynk.virtualWrite(VPIN_play, 1);   // Denote that vacuum cleaner is not in a cleaning mode
    Blynk.virtualWrite(VPIN_home, 1);   // Denote that vacuum cleaner is not in a cleaning mode
    delay(300);
    Serial.println("Eufy will go home!");
    terminal.println("Bedeep boop beep, going home");
  }
  
  if (Clean_button == 0 && ubh ==1){
    ubh = 0;
    irsend.sendRaw(ir_play, 18, khz);   // Send Play signal on press
    Blynk.virtualWrite(VPIN_play, 0);   // Denote that vacuum cleaner is not in a cleaning mode
    Blynk.virtualWrite(VPIN_home, 0);   // Denote that vacuum cleaner is not in a cleaning mode
    delay(300);
    Serial.println("Stop going home!");
    terminal.println("Stop going home!");
  }
}

//
BLYNK_WRITE(VPIN_play){
  int Clean_button = param.asInt();
  if (Clean_button == 1){
    irsend.sendRaw(ir_play, 18, khz);   // Send Play signal on press
    Blynk.virtualWrite(VPIN_play, 1);   // Denote that vacuum cleaner is in a cleaning mode
    delay(300);
    terminal.println("Resuming the robot");
    }
    else if (Clean_button == 0) {
    irsend.sendRaw(ir_pause, 18, khz);   // Send Pause signal on press
    Blynk.virtualWrite(VPIN_play, 0);   // Denote that vacuum cleaner is not in a cleaning mode
    delay(300);
    terminal.println("Pausing the robot");
    }
}

// Navigate using the D-PAD
BLYNK_WRITE(VPIN_dpad){
  float x_pos = param[0].asFloat();
  float y_pos = param[1].asFloat();
  int x = round(x_pos); //Rounding to get only directional answers
  int y = round(y_pos);

  if (x == 1 && y == 0){ // right
  
  //  while ( x == 1 && y == 0 ) {
    irsend.sendRaw(ir_right, 17, khz);   // Send Right signal on press
    //Blynk.virtualWrite(VPIN_play, 1);   // Denote that vacuum cleaner is in a cleaning mode
    //Blynk.virtualWrite(VPIN_play, 1);   // Denote that vacuum cleaner is in a cleaning mode
    terminal.println("Wooo...going right");
  //}
  
  }
  else if (x == -1 && y == 0){ //left
    irsend.sendRaw(ir_left, 17, khz);   // Send Left signal on press
    //Blynk.virtualWrite(VPIN_play, 1);   // Denote that vacuum cleaner is in a cleaning mode
    terminal.println("Wooo...going left");
  }
  else if (x == 0 && y == 1){ //up
    irsend.sendRaw(ir_up, 17, khz);   // Send Up signal on press
    //Blynk.virtualWrite(VPIN_play, 1);   // Denote that vacuum cleaner is in a cleaning mode
    terminal.println("Wooo...going up");
  }
  else if (x == 0 && y == -1){ //down
    irsend.sendRaw(ir_down, 17, khz);   // Send Down signal on press
    //Blynk.virtualWrite(VPIN_play, 1);   // Denote that vacuum cleaner is in a cleaning mode
    terminal.println("Wooo...going down");
  }
  /*else if (x == -1 && y == -1){ //left, down
    irsend.sendRaw(ir_down, 83, khz);   // Send Down signal on press
    irsend.sendRaw(ir_left, 83, khz);   // Send Left signal on press
    //Blynk.virtualWrite(VPIN_play, 1);   // Denote that vacuum cleaner is in a cleaning mode
    terminal.println("Wooo...going left-down");
  }*/
}

// Start cleaning in room mode
BLYNK_WRITE(VPIN_room){
  int Clean_button = param.asInt();
  if (Clean_button == 1){
    irsend.sendRaw(ir_room, 83, khz);   // Send Room Cleaning signal on press
    Blynk.virtualWrite(VPIN_play, 1);   // Denote that vacuum cleaner is in a cleaning mode
    terminal.println("Clean you room!");
  }
}

// Start cleaning in max mode
BLYNK_WRITE(VPIN_max){
  int Clean_button = param.asInt();
  if (Clean_button == 1 && ubm == 0){
    ubm = 1;
    irsend.sendRaw(ir_play, 18, khz);   // Send Spot Cleaning signal on press
    delay(300);
    irsend.sendRaw(ir_max, 54, khz);   // Send Max Cleaning signal on press
    Blynk.virtualWrite(VPIN_play, 1);   // Denote that vacuum cleaner is in a cleaning mode
    Blynk.virtualWrite(VPIN_max, 1);   // Denote that vacuum cleaner is in a cleaning mode
    Serial.println("Start maximized power");
    terminal.println("Cleaning for the queen!");
  }
  
  if (Clean_button == 0 && ubm == 1){
    ubm = 0;
    irsend.sendRaw(ir_play, 18, khz);   // Send Spot Cleaning signal on press
    Blynk.virtualWrite(VPIN_play, 0);   // Denote that vacuum cleaner is in a cleaning mode
    Blynk.virtualWrite(VPIN_max, 0);   // Denote that vacuum cleaner is in a cleaning mode
    Serial.println("Stop maximized power");
    terminal.println("Cleaning for the queen stop");
  }
}

// Start cleaning in wall mode
BLYNK_WRITE(VPIN_wall){
  int Clean_button = param.asInt();
  if (Clean_button == 1){
    irsend.sendRaw(ir_wall, 83, khz);   // Send Wall Cleaning signal on press
    Blynk.virtualWrite(VPIN_play, 1);   // Denote that vacuum cleaner is in a cleaning mode
    terminal.println("Wall clean!");
  }
}

// Start cleaning in spot mode
BLYNK_WRITE(VPIN_spot){
  int Clean_button = param.asInt();
  if (Clean_button == 1  && ub == 0){
    ub = 1;
    irsend.sendRaw(ir_play, 18, khz);   // Send Spot Cleaning signal on press
    delay(300);
    irsend.sendRaw(ir_spot, 54, khz);   // Send Spot Cleaning signal on press
    Blynk.virtualWrite(VPIN_spot, 1);   // Denote that vacuum cleaner is in a cleaning mode
    Blynk.virtualWrite(VPIN_play, 1);   // Denote that vacuum cleaner is not in a cleaning mode
    terminal.println("Spot clean!");
    Serial.println("Start spot\edge");

  }
  if (Clean_button == 0 && ub == 1){
    ub = 0;
    irsend.sendRaw(ir_play, 18, khz);   // Send Spot Cleaning signal on press
    Blynk.virtualWrite(VPIN_play, 0);   // Denote that vacuum cleaner is not in a cleaning mode
    Blynk.virtualWrite(VPIN_spot, 0);   // Denote that vacuum cleaner is in a cleaning mode
    Serial.println("Stop spot\edge");
    terminal.println("Spot clean! stop");
  }
}

// Start cleaning according to a timer
BLYNK_WRITE(VPIN_timerONOFF){
  int Timer_status = param.asInt();
  ts = param.asInt();
  Serial.print("VPIN_timerONOFF _ Timer_status : ");
  Serial.println(Timer_status);
}

// Start cleaning according to a timer, only if the timer status is set to be used
BLYNK_WRITE(VPIN_timer){
  int Clean_button = param.asInt();
  Serial.print("VPIN_timer _ Timer_status : ");
  Serial.println(Timer_status);
  Serial.print("VPIN_timer _ Clean_button : ");
  Serial.println(Clean_button);

  //if (Timer_status == 1 && Clean_button == 1){
  if (ts == 1 && Clean_button == 1){
    if (timergo == 0){
    timergo = 1;
    irsend.sendRaw(ir_play, 18, khz);   // Send Auto Clean signal on press
    delay(300);
    irsend.sendRaw(ir_auto, 54, khz);   // Send Auto Clean signal on press
    Blynk.virtualWrite(VPIN_play, 1);   // Denote that vacuum cleaner is in a cleaning mode
    Blynk.virtualWrite(VPIN_clean, 1);   // Denote that vacuum cleaner is in a cleaning mode
    delay(300);
    terminal.println("Started timer clean");
    Serial.println("Started timer clean");
    }
  }
  
  if (ts == 1 && Clean_button == 0){
    if (timergo == 1){
      timergo = 0;
      irsend.sendRaw(ir_play, 18, khz);   // Send Auto Clean signal on press
      delay(300);
      Blynk.virtualWrite(VPIN_play, 0);   // Denote that vacuum cleaner is in a cleaning mode
      Blynk.virtualWrite(VPIN_clean, 0);   // Denote that vacuum cleaner is in a cleaning mode
      terminal.println("finish timer clean");
      Serial.println("finish timer clean");
    }
  }
}

// Feed the watchdog!
void WDT(){
  ESP.wdtFeed();
}

// Connect wifi
void startWiFi() { // Start a Wi-Fi access point, and try to connect to some given access points. Then wait for either an AP or STA connection
  WiFi.begin(ssid, password);                 // Connect to the network
  Serial.print("Connecting to ");
  Serial.print(ssid); Serial.println(" ...");
  int i = 0;
  while (WiFi.status() != WL_CONNECTED) {     // Wait for the Wi-Fi to connect
    delay(1000);
    Serial.print(++i); Serial.print(' ');
    i = i++;
    if (i > 300){
      break; // Exit after 5 minutes
    }
  }
  Serial.println('\n');
  Serial.println("Connection established!");
  Serial.print("IP address:\t");
  terminal.println("Connection established, IP address:\t");
  terminal.print(WiFi.localIP());
  Serial.println(WiFi.localIP());             // Print IP address
}

// Start to OTA server
void startOTA() {                             // Start the OTA service
  ArduinoOTA.setHostname(OTAName);            // Hostname for OTA
  ArduinoOTA.setPassword(OTAPassword);        // Password for OTA

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
    terminal.println("Starting firmware flash");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\r\nOTA Flash complete");
    terminal.println("\r\nOTA Flash is complete");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    terminal.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA server ready\r\n");
}
