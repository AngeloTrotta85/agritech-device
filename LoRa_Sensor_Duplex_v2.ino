/*
  LoRa Duplex communication

  Sends a message every half second, and polls continually
  for new incoming messages. Implements a one-byte addressing scheme,
  with 0xFF as the broadcast address.

  Uses readString() from Stream class to read payload. The Stream class'
  timeout may affect other functuons, like the radio's callback. For an

  created 28 April 2017
  by Tom Igoe
*/

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>

//Libraries for OLED Display
#include <Wire.h>
#include <DFRobot_SHT20.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <SoftwareSerial.h>
#include <HardwareSerial.h>

//#define LILYGO_T3_V1_6

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

  //#define BOARD_LED                   25
  //#define LED_ON                      HIGH

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

  // RAIN SENSOR
  #define POWER_PIN 23  // ESP32's pin that provides the power to the rain sensor
  #define AO_PIN 2     // ESP32's pin connected to AO pin of the rain sensor

  #define DHTPIN 17
  #define DHTTYPE DHT22

  // NPK Sensor
  // RE and DE Pins set the RS485 module
  // to Receiver or Transmitter mode
  #define RE 21
  #define DE 25

  //#define SOFTS1 34
  //#define SOFTS2 35
  #define RXD2 12
  #define TXD2 13

#endif



//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 866E6

//OLED pins
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// deep sleep 
#define uS_TO_S_FACTOR 1000000  // conversion factor for micro seconds to seconds 
#define mS_TO_S_FACTOR 1000     // conversion factor for milli seconds to seconds 
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
DFRobot_SHT20 sht20;
DHT dht(DHTPIN, DHTTYPE);

//SoftwareSerial mod(SOFTS1,SOFTS2);
HardwareSerial NPKSerial ( 2 );
byte values[11];
const byte code[] =  {0x01, 0x03, 0x00, 0x1e, 0x00, 0x03, 0x65, 0xCD};
const byte nitro[] = {0x01, 0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};
const byte phos[] =  {0x01, 0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc};
const byte pota[] =  {0x01, 0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};

enum status
{ 
    BOOT, 
    SEND, 
    WAIT_REPLY,
    WAIT_TIMER 
};
typedef enum status status_t;

status_t mystatus = BOOT;

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xA1;     // address of this device
byte destination = 0x01;      // destination to send to
long lastSendTime = 0;        // last send time
//int interval = 10000;          // interval between sends
int interval_base_data = 30000;
int interval_base_wait_ack = 3000;

bool oledOn = false;
String display_sent = "";
String display_recv = "";
int millis_sent = 0;
int millis_recv = 0;

//int milliseconds_to_sleep = 5000;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool goDeep = false;



/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}


void updateDisplay() {
  if (oledOn) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("LORA SENSOR DUPLEX v2");

    display.setCursor(0,10);
    //display.setTextSize(1);
    display.print("LoRa sent ");
    display.print(millis_sent);
    display.setCursor(5,20);
    display.print(display_sent);

    display.setCursor(0,40);
    display.print("LoRa recv ");
    display.print(millis_recv);
    display.setCursor(5,50);
    display.print(display_recv);

    //display.print("Counter:");
    //display.setCursor(50,30);
    //display.print(counter);    

    display.display();  
  }
}

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  pinMode(POWER_PIN, OUTPUT);  // configure the power pin pin as an OUTPUT
  delay(100); //Take some time to open up the Serial Monitor

  digitalWrite(POWER_PIN, LOW);  // turn the rain sensor's power OFF

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

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

  delay(500);
  sht20.initSHT20(); // Init SHT20 Sensor
  delay(100);
  sht20.checkSHT20(); // Check SHT20 Sensor
  
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA SENSOR DUPLEX 2");
  display.display();

  dht.begin();

  delay(100);

  //mod.begin(9600);
  NPKSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  //NPKSerial.begin(4800, SERIAL_8N1, RXD2, TXD2);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);

  digitalWrite(RE, LOW);
  digitalWrite(DE, LOW);
  
  Serial.println("LoRa Sender Test");

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
  display.print("LoRa Initializing OK!");
  display.display();
  delay(1000);

  mystatus = SEND;

  if (!oledOn) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
}

void loop() {

  switch (mystatus) {
  case BOOT:
    // non dovrei essere qui
    mystatus = SEND;
    break;
  case SEND:
    {
      unsigned long begin_millis = millis();

      float soil_humd = sht20.readHumidity(); // Read Humidity
      float soil_temp = sht20.readTemperature(); // Read Temperature

      // Reading temperature or humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
      float air_humd = dht.readHumidity();
      // Read temperature as Celsius (the default)
      float air_temp = dht.readTemperature();

      /*
      byte val1,val2,val3;
      val1 = nitrogen();
      Serial.println("nitrogen " + val1);
      delay(250);
      val2 = phosphorous();
      Serial.println("phosphorous " + val2);
      delay(250);
      val3 = potassium();
      Serial.println("potassium " + val3);
      delay(250);
      */

      //checkNPK();

      // Read rain sensor 
      digitalWrite(POWER_PIN, HIGH);  // turn the rain sensor's power  ON
      delay(20);                      // wait 20 milliseconds
      int rain_value = analogRead(AO_PIN);
      digitalWrite(POWER_PIN, LOW);  // turn the rain sensor's power OFF

      String message = "T:";   // send a message
      message += soil_temp;
      message += ";H:";
      message += soil_humd;
      
      message += ";t:";
      message += air_temp;
      message += ";h:";
      message += air_humd;

      message += ";R:";
      message += rain_value;

      sendMessage(message);
      Serial.println("Sending " + message);

      lastSendTime = millis();            // timestamp the message
      //lastSendTime = begin_millis;            // timestamp the message
      //interval = interval_base + random(100);    

      display_sent = String(message.c_str());
      millis_sent = lastSendTime;
      updateDisplay();

      Serial.print("begin_millis: ");
      Serial.println(begin_millis);
      Serial.print("end: ");
      Serial.println(millis());

      mystatus = WAIT_REPLY;
    }
    break;

  case WAIT_REPLY:
    onReceive(LoRa.parsePacket());
    if (millis() - lastSendTime > interval_base_wait_ack){
      mystatus = WAIT_TIMER;
    }
    break;
  case WAIT_TIMER:
    if (goDeep) {
      /*
      First we configure the wake up source
      We set our ESP32 to wake up every 5 seconds
      */
      esp_sleep_enable_timer_wakeup((interval_base_data - interval_base_wait_ack) * mS_TO_S_FACTOR);
      Serial.println("Setup ESP32 to sleep for " + String(interval_base_data - interval_base_wait_ack) +
      " milliSeconds");

      /*
      Now that we have setup a wake cause and if needed setup the
      peripherals state in deep sleep, we can now start going to
      deep sleep.
      In the case that no wake up sources were provided but deep
      sleep was started, it will sleep forever unless hardware
      reset occurs.
      */
      Serial.println("Going to sleep now");
      Serial.flush(); 
      LoRa.sleep();
      delay(10);
      esp_deep_sleep_start();
    }
    else{
      onReceive(LoRa.parsePacket());
      if (millis() - lastSendTime > (interval_base_data)){
        mystatus = SEND;
      }
    }
    break;
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

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  display_recv = String(incoming.c_str());
  millis_recv = millis();
  updateDisplay();

  parseMessage(incoming);

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
}

void parseMessage(String rcvmsg) {
  int index = rcvmsg.indexOf(';');

  String command = rcvmsg.substring(2, index);

  Serial.println("Executing command: " + command);

  if (command == "0"){  // Just an Ack
    //nothing to do
    Serial.println("ACK received");

    return;
  }
  else if (command == "1") { //change sensing period
    Serial.println("COMMAND received: change-interval send");

    String value = rcvmsg.substring(6, rcvmsg.length());
    interval_base_data = value.toInt();
    if (interval_base_data < 3000){
      interval_base_data = 3000;
    }

  }
  else if (command == "2") { //change waiting ack period
    Serial.println("COMMAND received: change-interval reply");

    String value = rcvmsg.substring(6, rcvmsg.length());
    interval_base_wait_ack = value.toInt();
    if (interval_base_wait_ack < 3000){
      interval_base_wait_ack = 3000;
    }

  }
  else if (command == "3") { //change deep sleep
    Serial.println("COMMAND received: on/off deep sleep");

    String value = rcvmsg.substring(6, rcvmsg.length());
    int isdeep = value.toInt();

    goDeep = isdeep != 0;
  }
  else if (command == "4") { // on/off the OLED display
    Serial.println("COMMAND received: on/off OLED display");

    String value = rcvmsg.substring(6, rcvmsg.length());
    int ison = value.toInt();

    oledOn = ison != 0;

    if (oledOn) {
      display.ssd1306_command(SSD1306_DISPLAYON);
    }
    else{
      display.ssd1306_command(SSD1306_DISPLAYOFF);
    }
  }

}

void checkNPK(){
  byte val;
  int nitrogen;
  int phosphorous;
  int potassium;

  Serial.print("Checking NPK:    ");

  memset(values, 0, sizeof(values));
  NPKSerial.flush();

  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(10);

  for (uint8_t i = 0; i < sizeof(nitro); i++ ) NPKSerial.write( nitro[i] );
  NPKSerial.flush();

  // switching RS485 to receive mode
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  // delay to allow response bytes to be received!
  delay(200);

  // read in the received bytes
  for (byte i = 0; i < 7; i++) {
    values[i] = NPKSerial.read();
    Serial.print(values[i], HEX);
    Serial.print(' ');
  }
  Serial.println(' ');
  /*
  if (NPKSerial.write(nitro, sizeof(nitro)) == 8)
  {
    NPKSerial.flush();
    //delay(10);
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);
    delay(500);
    for (byte i = 0; i < 11; i++)
    {
      //Serial.print(mod.read(),HEX);
      values[i] = NPKSerial.read();
      Serial.print(values[i], HEX);
      Serial.print(" - ");
    }
    Serial.println();
  }
  nitrogen = values[4];
  phosphorous = values[6];
  potassium = values[8];
  Serial.print("nitrogen: ");
  Serial.println(nitrogen);
  Serial.print("phosphorous: ");
  Serial.println(phosphorous);
  Serial.print("potassium: ");
  Serial.println(potassium);
  */
}

byte nitrogen(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(NPKSerial.write(nitro,sizeof(nitro))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
    //Serial.print(mod.read(),HEX);
    values[i] = NPKSerial.read();
    Serial.print(values[i],HEX);
    }
    Serial.println();
  }
  return values[4];
}
 
byte phosphorous(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(NPKSerial.write(phos,sizeof(phos))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
    //Serial.print(mod.read(),HEX);
    values[i] = NPKSerial.read();
    Serial.print(values[i],HEX);
    }
    Serial.println();
  }
  return values[4];
}
 
byte potassium(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(NPKSerial.write(pota,sizeof(pota))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
    //Serial.print(mod.read(),HEX);
    values[i] = NPKSerial.read();
    Serial.print(values[i],HEX);
    }
    Serial.println();
  }
  return values[4];
}

