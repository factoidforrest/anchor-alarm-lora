#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <pins.h>
#include <LoraFunctionsRemote.h>
#include <AsyncElegantOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, RST_OLED);

#define VBAT_PIN 1
#define VBAT_READ_CNTRL_PIN 37 // Heltec GPIO to toggle VBatt read connection …
// Also, take care NOT to have ADC read connection
// in OPEN DRAIN when GPIO goes HIGH
#define ADC_READ_STABILIZE 10 // in ms (delay from GPIO control and ADC connections times)

#define ALARM_PIN 40

float readBatLevel() {
  int analogValue = analogRead(VBAT_PIN);
  float voltage = 0.0041 * analogValue;
  return voltage;
}


AsyncWebServer server(80);

void enableOTAUpdate() {
  WiFi.mode(WIFI_AP);

  WiFi.softAP("AnchorAlarmRemoteOTA");


  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! Head to /update to update the firmware. Use a .bin file from platformio build output");
  });
  
  AsyncElegantOTA.begin(&server);    // Start AsyncElegantOTA
  server.begin();
}

u_int32_t startupTime = millis();
bool serverOn = true;



void setup() {
  
  Serial.begin(115200);
  Serial.println("Booted");
  Wire.begin(SDA_OLED, SCL_OLED);

  enableOTAUpdate();

  // turn on vbat read
  pinMode(VBAT_READ_CNTRL_PIN, OUTPUT);
  digitalWrite(VBAT_READ_CNTRL_PIN, LOW);

  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, HIGH);
  pinMode(LED, OUTPUT);

  // initialize OLED
  delay(100);
  Serial.println("Awake");
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println("Starting...");
  display.display();
  bool lora_success = setup_lora();
  if (lora_success){
    display.println("Lora Chip Working");
  } else {
    display.println("Lora Chip Failure");
  }
  bool receive_success = setup_rx_mode();
  if (receive_success){
    display.println("Receive Working");
  } else {
    display.println("Receive Failure");
  }

  float vbat = readBatLevel();
  display.println("Batt: " + String(vbat) + "v");
  display.display();
  delay(500);

}


void setAlarm(bool alarmOn){
  Serial.println("setalarm called");
  if (alarmOn){
    Serial.println("Alarm on");
    digitalWrite(ALARM_PIN, LOW);
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(ALARM_PIN, HIGH);
    digitalWrite(LED, LOW);
  }
}

void handleWifiSleep(){
  // after 5 minutes shut off wifi
  if (serverOn && (millis() - startupTime) > 300000){
    serverOn = false;
    server.end();
    WiFi.mode( WIFI_OFF );
    btStop();
    // slow down the cpu, actual sleep is too hard but this is pretty good
    // if (!setCpuFrequencyMhz(40)){
    //   setCpuFrequencyMhz(80);
    // }
  }
}

String pBool(bool theBool){
  return theBool ? "True" : "False";
}

bool receivedPacketsPreviously = false;

uint lastReceivedTime;

ReceivedData lastPacket;

unsigned long packetCount = 0;

String errorText;

void loop() {
  handleWifiSleep();

  display.clearDisplay();
  display.setCursor(0, 0); 

  // handle packet
  ReceivedData received = receive_lora();
  if (received.error.length() > 0){

    if (received.error != "No Signal"){
      // handle error
      errorText = received.error;
    }
  } else {
    receivedPacketsPreviously = true;
    errorText = "";
    lastReceivedTime = millis();
    packetCount++;
    setAlarm(received.alarm);
    lastPacket = received;
  }

  // render display
  if (receivedPacketsPreviously){
    if (errorText != ""){
      display.println(errorText);
    }
    display.println("Armed:" + pBool(lastPacket.armed) + " Lim:"  + String(lastPacket.limit));
    display.println("Alarm: " + pBool(lastPacket.alarm));
    display.println("Dist: " + String(lastPacket.distance) + " " + lastPacket.direction);
    display.println("RSSI: " + String(lastPacket.rssi) + " SNR: " + String(lastPacket.snr));
    display.println("PacketNumber:" + String(packetCount));
    ulong timeSincePacket = (millis() - lastReceivedTime) / 1000;
    display.println("Time Since RX: " + String(timeSincePacket));
    
    if (timeSincePacket > 300){
      display.println("TIMEOUT!!!!");
      setAlarm(true);
    }

  } else {
    display.println("Waiting for signal...");
  }
  float vbat = readBatLevel();
  display.println("Batt: " + String(vbat) + "v");
  if (vbat< 3.5){
    setAlarm(true);
    display.println("LOW BATTERY");
  } 

  display.display();
  delay(500);
}
