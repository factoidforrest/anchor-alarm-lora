#include <TinyGPSPlus.h>
#include <ezButton.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_adc_cal.h"
#include <pwmWrite.h>
#include <AsyncElegantOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LoraFunctions.h>

#define OLED_I2C_ADDR 0x3C // Replace if needed

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, RST_OLED);


#define GPS_BAUD 9600


#define ROTARY_PIN_A 47
#define ROTARY_PIN_B 45
#define ROTARY_PIN_BTN 48

#define BUZZER_PIN 40

#define VBAT_PIN 1
#define VBAT_READ_CNTRL_PIN 37 // Heltec GPIO to toggle VBatt read connection …
// Also, take care NOT to have ADC read connection
// in OPEN DRAIN when GPIO goes HIGH

#define ALARM_PIN 40

float readBatLevel() {
  int analogValue = analogRead(VBAT_PIN);
  float voltage = 0.0041 * analogValue;
  return voltage;
}

#define MAX_DISTANCE_FT 2000 // maximum allowable distance

Pwm pwm = Pwm();

TinyGPSTime lastGpsLock = TinyGPSTime();

TinyGPSPlus gps;

double anchor_lat, anchor_lng;
bool is_armed = false;
ezButton button(ROTARY_PIN_BTN); 


AsyncWebServer server(80);

void enableOTAUpdate() {
  WiFi.mode(WIFI_AP);

  WiFi.softAP("AnchorAlarmOTA");


  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! Head to /update to update the firmware. Use a .bin file from platformio build output");
  });
  
  AsyncElegantOTA.begin(&server);    // Start AsyncElegantOTA
  server.begin();
}

u_int32_t startupTime = millis();
bool serverOn = true;

u_int32_t lastTouchedTime = millis();

int alarm_range = 250;

void read_encoder() {
  // cli();
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ROTARY_PIN_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ROTARY_PIN_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    alarm_range += 5;              // Increase counter
    encval = 0;
  }
  else if( encval < -3 ) {  // Four steps backwards
   alarm_range -=5;               // Decrease counter
   encval = 0;
  }
  // sei();
  lastTouchedTime = millis();
}

bool isAlarm = false;
bool lora_ok = false;

float Voltage = 0.0;

HardwareSerial GPS_Serial(1);


void setup() {
    Serial.begin(115200);
    Serial.println("Booted");
    GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, 6, 7);
    // Serial.println("HELLO WORLD, I HAVE BOOTED!!!!!!!!!!!!!!");

    Wire.begin(SDA_OLED, SCL_OLED);

    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
      // Serial.println(F("SSD1306 allocation failed"));
      for(;;);
    }
    display.setRotation(2);
    display.clearDisplay();

    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0,0);      // Start at top-left corner
    display.println(F("Hello, World!"));

    // Display static text
    display.println("Startup, waiting for GPS...");
    
    pinMode(BUZZER_PIN,OUTPUT);
    digitalWrite(BUZZER_PIN, HIGH);

    // turn on vbat read
    pinMode(VBAT_READ_CNTRL_PIN, OUTPUT);
    digitalWrite(VBAT_READ_CNTRL_PIN, LOW);

    enableOTAUpdate();

    lastTouchedTime = millis();
    // esp_sleep_enable_uart_wakeup(0);
    // gpio_wakeup_enable(GPIO_NUM_6, GPIO_INTR_LOW_LEVEL);
    // gpio_wakeup_enable(GPIO_NUM_7, GPIO_INTR_LOW_LEVEL);
    // gpio_wakeup_enable(GPIO_NUM_10, GPIO_INTR_LOW_LEVEL);

    // esp_sleep_enable_gpio_wakeup();

    pinMode(ROTARY_PIN_A, INPUT_PULLUP);
    pinMode(ROTARY_PIN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_A), read_encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_B), read_encoder, CHANGE);

    pinMode(ROTARY_PIN_BTN, INPUT_PULLUP);
    
    button.setDebounceTime(50); // set debounce time to 50 milliseconds
    Voltage = readBatLevel();
    display.println(String("VBat: " + String(Voltage) + "v").c_str());

    lora_ok = setup_lora();
    display.println(lora_ok ? "Lora Ok" : "Lora FAILURE");
    display.display(); 
    delay(1000);

}

double calculateBearing(double lat1, double long1, double lat2, double long2) {
  lat1 = lat1 * (M_PI / 180);
  long1 = long1 * (M_PI / 180);
  lat2 = lat2 * (M_PI / 180);
  long2 = long2 * (M_PI / 180);

  double dLon = (long2 - long1);
  double x = cos(lat2) * sin(dLon);
  double y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

  double bearing = atan2(x, y);
  bearing = bearing * (180 / M_PI);
  bearing = (int)(bearing + 360) % 360;

  return bearing;
}

String cardinalDirection(double bearing) {
  const char* directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  int index = (int)((bearing + 11.25) / 22.5) % 16;
  return directions[index];
}


bool sentResult = false;

unsigned long lastSent = 0;

void displayGPS() {
  display.clearDisplay();
  display.setCursor(0, 0);

  display.println(String("Alarm: " + String(is_armed ? "ARMED" : "Disarmed")).c_str());
  display.println("Locked: " + String(gps.satellites.value()));
  display.println("Accuracy(meters): " + String(gps.hdop.value()));

  Voltage = readBatLevel();
  display.println(String("VBat: " + String(Voltage) + "v").c_str());

  display.println(String("Range(feet): " + String(alarm_range) + "ft").c_str());

  double current_distance = 0;
  double bearing = 0;
  String direction = "";
  if (is_armed) {
    current_distance = TinyGPSPlus::distanceBetween( anchor_lat, anchor_lng, gps.location.lat(), gps.location.lng() );
    current_distance *= 3.281; // convert to feet

    bearing = calculateBearing(anchor_lat, anchor_lng, gps.location.lat(), gps.location.lng());
    direction = cardinalDirection(bearing);
    
    display.println(String("Dist: " + String(current_distance, 2) + "ft " + direction).c_str());

  }

  uint8_t txPower = Voltage > 3.7 ? 16 : 12;
  if (lastSent == 0 || (millis() - lastSent) > 10000){
  sentResult = transmit_lora_packet(22, current_distance,direction,alarm_range,is_armed, is_armed && isAlarm);
    lastSent = millis();
  }
  if (lora_ok){
    display.print("Sent ");
    display.print((millis() - lastSent) / 1000);
    display.print("s ago");
  } else {
    display.print("Lora FAILURE");
  }

  display.print("TX:");
  display.println(sentResult ? "Ok" : "Fail");
  display.display();

}


void alarm(bool isOn){
  isAlarm = isOn;
  digitalWrite(BUZZER_PIN, isOn ? LOW : HIGH);//2000, isOn ? 65535 : 0);
}

void checkDistance() {
  if (is_armed) {
    double current_distance = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), anchor_lat, anchor_lng);
    current_distance *= 3.281; // convert to feet
      if (current_distance > alarm_range) {
      alarm(true);
    } else if (Voltage < 3.300) {

      alarm(true);
    } else {
      alarm(false);
    }


  } else {
    alarm(false);
  }
}



void lockLost() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("GPS Lock: No");
  display.println("Sattelites: " + String(gps.satellites.value()));
  Voltage = readBatLevel();
  display.print("VBat:");
  display.println(Voltage);

  display.display();
  if (is_armed) {
    alarm(true);
  }
}

void handleDisplaySleep(){

  if ((millis() - lastTouchedTime) > 120000){
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  } else {
    display.ssd1306_command(SSD1306_DISPLAYON);
  }
}

void handleWifiSleep(){
  // after 5 minutes shut off wifi
  // if (serverOn && (millis() - startupTime) > 300000){
  //   serverOn = false;
  //   server.end();
  //   WiFi.mode( WIFI_OFF );
  //   btStop();
  //   // slow down the cpu, actual sleep is too hard but this is pretty good
  //   if (!setCpuFrequencyMhz(40)){
  //     setCpuFrequencyMhz(80);
  //   }
  // }
}

void loop() {
  button.loop();
  

  handleDisplaySleep();

  handleWifiSleep();

  if (lastGpsLock.isValid() && lastGpsLock.age() > 10000 && is_armed){
    lockLost();
  }






  while (GPS_Serial.available() > 0){
    if (gps.encode(GPS_Serial.read())) {
      if (!gps.location.isValid() && lastGpsLock.age() > 10000) {
          lockLost();
      } else {
        lastGpsLock = gps.time;
        if (button.getCount() > 0) {
          lastTouchedTime = millis();
          button.resetCount();
          if (is_armed) {
            is_armed = false;
          } else {
            is_armed = true;
            anchor_lat = gps.location.lat();
            anchor_lng = gps.location.lng();
          }
        }
        displayGPS();
        checkDistance();
      }
    } else {
        display.clearDisplay();
        display.println("GPS Error");
    }
  }

}


