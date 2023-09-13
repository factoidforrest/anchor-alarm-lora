#ifndef LORA_FUNCTIONS_H
#define LORA_FUNCTIONS_H
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
bool transmit_lora_packet(int power, double distance, String direction, int limit, boolean armed, boolean alarm);
ReceivedData receive_lora();

#endif // LORA_FUNCTIONS_H