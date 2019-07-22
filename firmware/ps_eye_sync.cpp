#include "Arduino.h"

uint32_t dT_avg  = 0;
uint32_t avg_acc = 0;
uint32_t counter = 0;
uint8_t  avg_cnt = 1;

uint16_t offset  = 100;

bool triggered = false;
bool rising    = false;
bool even      = true;

void vsync_int(){
  triggered = true;
  rising = digitalRead(2);
}

void setup() {
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
  attachInterrupt(digitalPinToInterrupt(2), vsync_int, CHANGE);
}

void loop() {

  if ((counter+offset == dT_avg/2)&&even){
    digitalWrite(6, LOW);
  }

  if (triggered && rising){
    even = !even;
    if (even) digitalWrite(6, HIGH);
    avg_acc += counter;
    if (avg_cnt == 10) {
      dT_avg  = avg_acc / 10;
      avg_cnt = 1;
      avg_acc = 0;
      offset  = dT_avg/16;
    }
    avg_cnt++;
    counter = 0;
    triggered = false;
  }

  counter++;
}
