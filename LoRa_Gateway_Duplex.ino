/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/ttgo-lora32-sx1276-arduino-ide/
*********/

#include <WiFi.h>
#include <PubSubClient.h>

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define LILYGO_T3_V1_6
#define UNUSE_PIN                   (0)

#if defined(LILYGO_T3_V1_6)
  #define OLED_SDA                     21
  #define OLED_SCL                     22
  #define OLED_RST                    UNUSE_PIN

  #define RADIO_SCLK_PIN              5
  #define RADIO_MISO_PIN              19
  #define RADIO_MOSI_PIN              27
  #define RADIO_CS_PIN                18
  #define RADIO_DIO0_PIN               26
  #define RADIO_RST_PIN               23
  #define RADIO_DIO1_PIN              33
  #define RADIO_BUSY_PIN              32

  //VARIABLES form the other example
  #define SCK 5
  #define MISO 19
  #define MOSI 27
  #define SS 18
  #define RST 23
  #define DIO0 26

  #define SDCARD_MOSI                 15
  #define SDCARD_MISO                 2
  #define SDCARD_SCLK                 14
  #define SDCARD_CS                   13

  #define BOARD_LED                   25
  #define LED_ON                      HIGH

  #define ADC_PIN                     35

#else

  //define the pins used by the LoRa transceiver module
  #define SCK 5
  #define MISO 19
  #define MOSI 27
  #define SS 18
  #define RST 14
  #define DIO0 26

  //OLED pins
  #define OLED_SDA 4
  #define OLED_SCL 15 
  #define OLED_RST 16

#endif

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 866E6


typedef struct recv_header {
  byte destination_addr;                 // destination address
  byte source_addr;                      // sender address
  byte msgID;                            // message ID [0..255]
  byte msgLen;                            // payload length
} recv_header_t;

// Replace the next variables with your SSID/Password combination
const char* ssid = "wilmalab";
const char* password = "wilmalab-wifi!";

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "192.168.0.178";
//const char* mqtt_server = "YOUR_MQTT_BROKER_IP_ADDRESS";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

String LoRaData;

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0x01;     // address of this device
byte destination = 0xA1;      // destination to send to 0xFF is bradcast (0xA1 is the LoRa device in the garden)

String display_sent = "";
int millis_sent = 0;
String display_recv = "";
int millis_recv = 0;

void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("LORA GATEWAY DUPLEX");

  display.setCursor(0,10);
  //display.setTextSize(1);
  display.print("LoRa sent ");
  display.print(millis_sent);
  display.setCursor(10,20);
  display.print(display_sent);

  display.setCursor(0,30);
  display.print("LoRa recv ");
  display.print(millis_recv);
  display.setCursor(10,40);
  display.print(display_recv);

  //display.print("Counter:");
  //display.setCursor(50,30);
  //display.print(counter);    

  display.display();  
}

void onCadDone(boolean signalDetected) {
  if (signalDetected){
    Serial.println("signalDetected");
  }
  else {
    Serial.println("NOT signalDetected");
  }
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
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
}

void callback_mqtt(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "/loragw/command") {
    /*Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }*/
  }
  else if (String(topic) == "/loragw/commandnow") {
    sendMessage(messageTemp);

    Serial.print("Sending message direct now: ");
    Serial.println(messageTemp);
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("LoRaGatewayClient")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("/loragw/command");
      client.subscribe("/loragw/commandnow");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() { 
  //initialize Serial Monitor
  Serial.begin(115200);
  
  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  
  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA GATEWAY DUPLEX");
  display.display();

  Serial.println("LoRa Receiver Test");
  
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
  display.setCursor(0,10);
  display.println("LoRa Initializing OK!");
  display.display();  

  //LoRa.onCadDone(onCadDone);

  //LoRa.channelActivityDetection();

  //LoRa.dumpRegisters(Serial);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback_mqtt);

  display.setCursor(0,20);
  display.println("MQTT Initializing OK!");
  display.display();  
}


void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // parse for a packet, and call onReceive with the result:
  recv_header_t rcv_h;
  if (onReceive(LoRa.parsePacket(), &rcv_h)) {

    //delay(100);

    String message = "C:0;V:0";   // send a message
    //String message = "C:1;V:5000";   // send a message
    sendMessageDirect(message, rcv_h.source_addr);
    //sendMessage(message);

    Serial.print("Sending message: ");
    Serial.println(message);

    display_sent = message;
    millis_sent = millis();
    updateDisplay();
  }

}

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

void sendMessageDirect(String outgoing, byte dest) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(dest);                     // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

bool onReceive(int packetSize, recv_header_t *rcv_h) {
  if (packetSize == 0) return false;          // if there's no packet, return

  // read packet header bytes:
  byte recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return false;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return false;                             // skip rest of function
  }

  rcv_h->destination_addr = recipient;
  rcv_h->source_addr = sender;
  rcv_h->msgID = incomingMsgId;
  rcv_h->msgLen = incomingLength;

  display_recv = incoming;
  millis_recv = millis();
  updateDisplay();

  float soil_temp = 0;
  float soil_humd = 0;
  float air_temp = 0;
  float air_humd = 0;
  int rain = 0;
  sscanf( incoming.c_str(), 
          "T:%f;H:%f;t:%f;h:%f;R:%d", 
          &soil_temp, &soil_humd, &air_temp, &air_humd, &rain);

  char buff_str[8];

  snprintf(buff_str, 8, "%.2f", soil_temp);
  client.publish("/lora/sensors/soil_temperature", buff_str);

  snprintf(buff_str, 8, "%.2f", soil_humd);
  client.publish("/lora/sensors/soil_humidity", buff_str);

  snprintf(buff_str, 8, "%.2f", air_temp);
  client.publish("/lora/sensors/air_temperature", buff_str);

  snprintf(buff_str, 8, "%.2f", air_humd);
  client.publish("/lora/sensors/air_humidity", buff_str);

  snprintf(buff_str, 8, "%d", rain);
  client.publish("/lora/sensors/rain", buff_str);

  snprintf(buff_str, 8, "%d", LoRa.packetRssi());
  client.publish("/lora/sensors/rssi", buff_str);

  snprintf(buff_str, 8, "%.2f", LoRa.packetSnr());
  client.publish("/lora/sensors/snr", buff_str);

  /*
  char s_temp_c[8];
  char s_humd_c[8];
  char a_temp_c[8];
  char a_humd_c[8];
  char rain_c[8];
  snprintf(s_temp_c, 8, "%.2f", soil_temp);
  snprintf(s_humd_c, 8, "%.2f", soil_humd);
  snprintf(a_temp_c, 8, "%.2f", air_temp);
  snprintf(a_humd_c, 8, "%.2f", air_humd);
  snprintf(rain_c, 8, "%d", rain);
  client.publish("/lora/sensors/soil_temperature", s_temp_c);
  client.publish("/lora/sensors/soil_humidity", s_humd_c);
  client.publish("/lora/sensors/air_temperature", a_temp_c);
  client.publish("/lora/sensors/air_humidity", a_humd_c);
  client.publish("/lora/sensors/rain", rain_c);
  */


  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();

  return true;
}
