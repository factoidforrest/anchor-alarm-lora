#ifndef LORA_FUNCTIONS_H
#define LORA_FUNCTIONS_H
#include <SX126XLT.h>                           //include the appropriate library  
#include <ArduinoJson.h>

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

boolean setup_lora();
boolean setup_rx_mode();
uint8_t transmit_lora(int distance, int direction, int limit, boolean armed, boolean alarm);
ReceivedData receive_lora();

#endif // LORA_FUNCTIONS_H