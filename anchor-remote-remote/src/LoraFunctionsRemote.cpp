#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>                                //the lora device is SPI based so load the SPI library                                         
#include <ArduinoJson.h>
// #include <LoraFunctions.h>
// #include <SX126XLT.h>     
// SX126XLT LT;                                                   //create a library class instance called LT
#include <RadioLib.h>

#define LoRa_MOSI 10
#define LoRa_MISO 11
#define LoRa_SCK 9

#define LoRa_nss 8
#define LoRa_dio1 14
#define LoRa_nrst 12
#define LoRa_busy 13

SX1262 radio = new Module(LoRa_nss, LoRa_dio1, LoRa_nrst, LoRa_busy);


// #define NSS 10                                  //select pin on LoRa device
// #define NRESET 9                                //reset pin on LoRa device
// #define RFBUSY 7                                //SX126X busy pin
// #define DIO1 3                                  //DIO1 pin on LoRa device, used for sensing RX and TX done 
// #define SW 5                                    //SW pin on LoRa device, used to power antenna switch
#define LORA_DEVICE DEVICE_SX1262               //we need to define the device we are using


uint8_t TXPacketL;
uint32_t TXPacketCount;


struct ReceivedData {
  int distance;
  String direction;
  int limit;

  bool armed;
  bool alarm;
  int rssi;
  int snr;
  String error;
};


boolean setup_lora()
{
  Serial.begin(9600);
  SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_nss);

  // initialize SX1262 with default settings
  Serial.print(F("[SX1262] Initializing ... "));
  int state = radio.begin(915.0, 125, 8U, 7U,19U,22);
  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("success!"));
    return true;
  }
  else
  {
    Serial.print(F("failed, code "));
    Serial.println(state);
    return false;
  }
}

volatile bool receivedFlag = false;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}

boolean setup_rx_mode(){
  radio.setPacketReceivedAction(setFlag);

  // start listening for LoRa packets
  Serial.print(F("[SX1262] Starting to listen ... "));
  int state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
    return true;
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    return false;
  }
}




bool hasSentOnePacket = false;

uint8_t transmit_lora_packet(int power, double distance, String direction, int limit, boolean armed, boolean alarm) {

  String sendStr; 

  StaticJsonDocument<256> doc;
  doc["distance"] = distance;
  doc["direction"] = direction;
  doc["limit"] = limit;
  doc["armed"] = armed;
  doc["alarm"] = alarm;

  serializeJson(doc, sendStr);  // Serialize the JSON object into the buffer
  int state = radio.transmit(sendStr);

  hasSentOnePacket = true;


  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F("success!"));

    // print measured data rate
    Serial.print(F("[SX1262] Datarate:\t"));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));
    return true;

  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));
    return false;

  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occured while transmitting packet
    Serial.println(F("timeout!"));
    return false;
  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
    return false;

  }

}



uint8_t RXBUFFER[256];

ReceivedData parse_packet(String receivedStr)
{


  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, receivedStr);

  if (error) {
    ReceivedData errorStruct;
    errorStruct.error = "PrsErr:" + String(error.f_str());
    return errorStruct;
  }


  int rssi = radio.getRSSI();
  int snr = radio.getSNR();

  ReceivedData received = {
    doc["distance"],
    doc["direction"],
    doc["limit"],
    doc["armed"],
    doc["alarm"],
    rssi,
    snr,
    ""
  };

  return received;
}




ReceivedData receive_lora()
{

  // you can receive data as an Arduino String
  // NOTE: receive() is a blocking method!
  //       See example ReceiveInterrupt for details
  //       on non-blocking reception method.

  if(!receivedFlag) {
    ReceivedData errorStruct;
    errorStruct.error = "No Signal";
    return errorStruct;
  }
  // reset flag
  receivedFlag = false;

  String receivedStr;
  int state = radio.readData(receivedStr);
  // you can also receive data as byte array
  /*
    byte byteArr[8];
    int state = radio.receive(byteArr, 8);
  */

  if (state == RADIOLIB_ERR_NONE)
  {
    // packet was successfully received
    Serial.println(F("success!"));

    // print the data of the packet
    Serial.print(F("[SX1262] Data:\t\t"));
    Serial.println(receivedStr);

    // print the RSSI (Received Signal Strength Indicator)
    // of the last received packet
    Serial.print(F("[SX1262] RSSI:\t\t"));
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));

    // print the SNR (Signal-to-Noise Ratio)
    // of the last received packet
    Serial.print(F("[SX1262] SNR:\t\t"));
    Serial.print(radio.getSNR());
    Serial.println(F(" dB"));

    return parse_packet(receivedStr);

  }
  else if (state == RADIOLIB_ERR_RX_TIMEOUT)
  {
    // timeout occurred while waiting for a packet
    Serial.println(F("timeout!"));
    ReceivedData errorStruct;
    errorStruct.error = "Timeout";
    return errorStruct;
  }
  else if (state == RADIOLIB_ERR_CRC_MISMATCH)
  {
    // packet was received, but is malformed
    Serial.println(F("CRC error!"));
        ReceivedData errorStruct;
    errorStruct.error = "crc err";
    return errorStruct;
  }
  else
  {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
    ReceivedData errorStruct;
    errorStruct.error = "Unknown Err";
    return errorStruct;
  }
  
}


