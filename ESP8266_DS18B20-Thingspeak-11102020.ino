/*  ESP8266, DS18B20 with updates to ThinkSpeak.com
 *   
 *  Supports multiple DS18B20 sensors on one pin, see Dallas datasheet for 
 *  wiring schematics.
 *  
 *  Requires free ThingSpeak account: https://thingspeak.com/
 *  Requires the following libraries to be installed: 
 *  
 *  OneWire.h - Built in.
 *  ESP8266Wifi.h - Built in to ESP8266/Arduino integration.
 *  DallasTemperature.h - Dallas Temperature sensor  library by Miles Burton: 
 *  https://github.com/milesburton/Arduino-Temperature-Control-Library 
 *  ThingSpeak.h - Offical ThinkSpeak library by Mathworks:
 *  https://github.com/mathworks/thingspeak-arduino
 *  
 *  Portions of code taken from Dallas Temperature libray examples by Miles Burton.
 *  
 *  To test sensor readings without uploading to ThinkSpeak cooment out 
 *  line 144 (ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);)
 *  
 */

#include <DallasTemperature.h>
#include <OneWire.h>
#include <SimpleThingspeak.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "ESPAsyncWebServer.h"


//----------------------------------------------------------------------------
// !!! remark THINGSPEAK_SERVER if not used THINGSPEAK !!!
#define THINGSPEAK_SERVER         "api.thingspeak.com" // api.thingspeak.com  
#define THINGSPEAK_PORT           80
#define THINGSPEAK_API_KEY        "WXN6GWY1HFRUWIF9"   //"THINGSPEAK_API_KEY";
#define THINGSPEAK_SEND_INTERVAL  2   /* in ms */

#ifdef THINGSPEAK_SERVER
SimpleThingspeak  thingspeak(THINGSPEAK_SERVER, THINGSPEAK_PORT, THINGSPEAK_API_KEY );
#endif

// Set to true to define Relay as Normally Open (NO)
#define RELAY_NO    true /*normal open*/

// Set number of relays
#define NUM_RELAYS  1

// Assign each GPIO to a relay
int relayGPIOs[NUM_RELAYS] = {16};


// User changeable vaules go here.

#define ONE_WIRE_BUS D4                           // Digital pin DS18B20 is connected to.
#define TEMPERATURE_PRECISION 12                  // Set sensor precision.  Valid options 8,10,11,12 Lower is faster but less precise

//For the vibration sensor (defs):
//=================================================================================================================================================

//=================================================================================================================================================
const int relayPin = D0;

//unsigned long myChannelNumber = 1035422;            // Thingspeak channel ID here
//const char * myWriteAPIKey = "M25YYA2L5IJ9SFCJ";  // Write API key here
//const char* ssid     = "Vodafone-8F14";           // SSID of wireless network
//const char* password = "F6Q34pP6LCMQBdnQ";       // Password for wireless network

const char* ssid     = "Dan2";           // SSID of wireless network
const char* password = "12331244";       // Password for wireless network

const char* mqtt_server = "192.168.3.12";
const char* mqttTopic = "dbt/byotov/statusRelay";

const char* PARAM_INPUT_1 = "relay";  
const char* PARAM_INPUT_2 = "state";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 3.0rem;}
    p {font-size: 3.0rem;}
    body {max-width: 600px; margin:0px auto; padding-bottom: 25px;}
    .switch {position: relative; display: inline-block; width: 120px; height: 68px} 
    .switch input {display: none}
    .slider {position: absolute; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; border-radius: 34px}
    .slider:before {position: absolute; content: ""; height: 52px; width: 52px; left: 8px; bottom: 8px; background-color: #fff; -webkit-transition: .4s; transition: .4s; border-radius: 68px}
    input:checked+.slider {background-color: #2196F3}
    input:checked+.slider:before {-webkit-transform: translateX(52px); -ms-transform: translateX(52px); transform: translateX(52px)}
  </style>
</head>
<body>
  <h2>ESP Web Server</h2>
  %BUTTONPLACEHOLDER%
<script>function toggleCheckbox(element) {
  var xhr = new XMLHttpRequest();
  if(element.checked){ xhr.open("GET", "/update?relay="+element.id+"&state=1", true); }
  else { xhr.open("GET", "/update?relay="+element.id+"&state=0", true); }
  xhr.send();
}</script>
</body>
</html>
)rawliteral";

// Replaces placeholder with button section in your web page
String processor(const String& var){
  //Serial.println(var);
  if(var == "BUTTONPLACEHOLDER"){
    String buttons ="";
    for(int i=1; i<=NUM_RELAYS; i++){
      String relayStateValue = relayState(i);
      buttons+= "<h4>Relay #" + String(i) + " - GPIO " + relayGPIOs[i-1] + "</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"" + String(i) + "\" "+ relayStateValue +"><span class=\"slider\"></span></label>";
    }
    return buttons;
  }
  return String();
}

String relayState(int numRelay){
  if(RELAY_NO){
    if(digitalRead(relayGPIOs[numRelay-1])){
      return "";
    }
    else {
      return "checked";
    }
  }
  else {
    if(digitalRead(relayGPIOs[numRelay-1])){
      return "checked";
    }
    else {
      return "";
    }
  }
  return "";
}

int fieldStart = 1;                               // Field number to start populating ThingSpeak channel data, leave at 
                                                  // 1 if this is the only device reporting to that channel.  
                                                  // If more than one device this should be the FIRST FREE field.

int updatePeriod = 2;                            //delay in seconds to update to ThingSpeak.  Should be set to not less than 15.

uint32_t lastexecutednow; 
uint32_t lastexecutednow2;



int status = WL_IDLE_STATUS;
WiFiClient  client;
PubSubClient mqtt(client);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
long lastMsg = 0;
char msg[100];
float temp;
float temp2;
int value = 0;

int numberOfDevices; // Number of temperature devices found
DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

uint8_t oldvalue;
bool publishtomqtt;

void relay_Set (uint8 newvalue) {
  if (oldvalue != newvalue){
    digitalWrite(relayPin, newvalue);
    oldvalue = newvalue;
    publishtomqtt = true;
    Serial.print(" Device " );
    Serial.println(newvalue ? "ON":"OFF");
  }
}

void callback(char* topic, byte* payload, unsigned int length) {

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  bool turnOnRelay = payload[0] == '1';
  if (turnOnRelay)
  {
    relay_Set (LOW);
  }
  else
  {
    relay_Set (HIGH);
  }

  Serial.println();


  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqtt.connect(clientId.c_str(), "dango" ,"1021358637")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //mqtt.publish("outTopic", "hello world");
      // ... and resubscribe
      mqtt.subscribe("inTopic");
      mqtt.subscribe(mqttTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



void setup() {
 Serial.begin(115200);
  delay(10);

  // Set all relays to off when the program starts - if set to Normally Open (NO), the relay is off when you set the relay to HIGH
  //for(int i=1; i<=NUM_RELAYS; i++){
   // pinMode(relayGPIOs[i-1], OUTPUT);
    //if(RELAY_NO){
      //digitalWrite(relayGPIOs[i-1], HIGH);
    //}
    //else{
      //digitalWrite(relayGPIOs[i-1], LOW);
    //}
  //}

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP()); 

  // Print ESP8266 Local IP Address
  Serial.println(WiFi.localIP());

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/update?relay=<inputMessage>&state=<inputMessage2>
  server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    String inputParam;
    String inputMessage2;
    String inputParam2;
    // GET input1 value on <ESP_IP>/update?relay=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1) & request->hasParam(PARAM_INPUT_2)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      inputParam = PARAM_INPUT_1;
      inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
      inputParam2 = PARAM_INPUT_2;
      
      if (inputMessage == "1") {
        relay_Set(!inputMessage2.toInt());
        Serial.println("relay_Set");
      }
      
      if(RELAY_NO){
        Serial.print("NO ");
        digitalWrite(relayGPIOs[inputMessage.toInt()-1], !inputMessage2.toInt());
      }
      else{
        Serial.print("NC ");
        digitalWrite(relayGPIOs[inputMessage.toInt()-1], inputMessage2.toInt());
      }
    }
    else {
      inputMessage = "No message sent";
      inputParam = "none";
    }
    Serial.println(inputMessage + inputMessage2);
    request->send(200, "text/plain", "OK");
  });
  // Start server
  server.begin();

  NTP_setup();  // MyNTP.ino
  
  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(callback);

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);

  //pinMode(VIBRATION_SENSOR_PIN, INPUT_PULLUP);

  vibration_setup();
  
  //ThingSpeak.begin(client);
  sensors.begin();
  
  // Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();
  
  // locate devices on the bus
  Serial.print("Locating devices...");
  
  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  
  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++)
  {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i))
  {
    Serial.print("Found device ");
    Serial.print(i, DEC);
    Serial.print(" with address: ");
    printAddress(tempDeviceAddress);
    Serial.println();
    
    Serial.print("Setting resolution to ");
    Serial.println(TEMPERATURE_PRECISION, DEC);
    
    // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
    sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
    
    Serial.print("Resolution actually set to: ");
    Serial.print(sensors.getResolution(tempDeviceAddress), DEC); 
    Serial.println();
  }else{
    Serial.print("Found ghost device at ");
    Serial.print(i, DEC);
    Serial.print(" but could not detect address. Check power and cabling");
  }
  }

  publishtomqtt = true;

}

//For the vibration sensor:

//=====================================================================================================================================

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void loop() {
  
  NTP_handle();
  
  if (!mqtt.connected()) {
    reconnect();
  }
  mqtt.loop();
  
  long now = millis();
  
  //===============================================================================================================================
  if (now > (lastexecutednow + 2000)) {
    lastexecutednow = now;
  sensors.requestTemperatures();
  // Itterate through each sensor and send readings to ThinkSpeak.  
  for (int i=0; i<=(numberOfDevices - 1); i++){
    if (i==0)    temp=sensors.getTempCByIndex(i);
    if (i==1)    temp2=sensors.getTempCByIndex(i);
    
    //ThingSpeak.setField(i+fieldStart,temp); //for the fields in the thingspeak 1,2,3,4,5...
    //Serial.println("Sensor #:");
    //Serial.println(i);
    //Serial.println("Temperature:");
    //Serial.println(temp);
    
  }
  //if (temp2 > 29.0) {
  //  digitalWrite(relayPin, HIGH);
  //}
  //else {    
  //digitalWrite(relayPin, LOW);
  //}
}
  //===============================================================================================================================
  if (now > (lastexecutednow2 + 30000)) {
    lastexecutednow2 = now;

   Serial.printf_P("[thingspeak][%li s] ...\n", (now/1000) );
    //thingspeak.send_now("","","","","","","","");
    thingspeak.send_now(   /*field1*/ String( temp  ),
                           /*field2*/ String( temp2  ),
                           /*field3*/ "",
                           /*field4*/ "",
                           /*field5*/ "",
                           /*field6*/ "",
                           /*field7*/ "",
                           /*field8*/ ""
                       );
    
  //ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);  //Write fields to Thingspeak, comment this line out
                                                              //if you wish to test without uploading data.
    //Serial.println("Data sent to ThinkSpeak");
 }
  
//===============================================================================================================================
  if (now - lastMsg > 2000) {
    lastMsg = now;
    uint32_t ts = (uint32_t)time(nullptr);
    
    ++value;

    //time_t now;
    //now = time(nullptr);
    //const tm* tm = localtime(&now);
    
    snprintf (msg, 100, "{"
                          "\"timestamp\" : %u," 
                          "\"temp\" : %.2f,"
                          "\"temp2\" : %.2f"
                          "}", ts, temp, temp2);
    //snprintf (msg1, 50, "{"
                          //"\"temp2\" : %f"
                          //"}", temp2);
    //Serial.print("Publish message: ");
    //Serial.println(msg);
    //Serial.println(msg1);
    mqtt.publish("dbt1/byotov/DS18B20", msg);
//================================================================================================================================
   vibration_loop();
//================================================================================================================================  
  } 
  if (publishtomqtt) {
    publishtomqtt = false;
    String msgMQTT = digitalRead (relayPin) ? "1":"0";
    mqtt.publish ("dbt1/byotov/statusRelay", msgMQTT.c_str());
    Serial.print("mqttpublish ");
    Serial.println(msgMQTT);
  }
  //delay(updatePeriod * 1000);
}
