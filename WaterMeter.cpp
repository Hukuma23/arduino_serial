#include "WaterMeter.h"

WaterMeter::WaterMeter(byte pin, float tolerance /*= 0.08*/) {
  this->pin = pin;
  
  pinMode(pin, INPUT);
  
  float low_level = 1023.0 * (R1/(R1+R3));
  low_min = low_level * (1 - tolerance);
  low_max = low_level * (1 + tolerance);
}

void WaterMeter::init() {
  
  int curr = analogRead(pin);

  if ((curr >= low_min) && (curr <= low_max)) {
    cntr_on = false;
  }
  else {
    cntr_on = true;
  }

  float low_level = 1023.0 * (R1/(R1+R3));

  DEBUG_PRINT("low_level=");
  DEBUG_PRINTLN(low_level);

  DEBUG_PRINT("low_min=");
  DEBUG_PRINTLN(low_min);
  
  DEBUG_PRINT("low_max=");
  DEBUG_PRINTLN(low_max);

  DEBUG_PRINT(pin);
  DEBUG_PRINT("  curr=");
  DEBUG_PRINTLN(curr); 

  DEBUG_PRINT("cntr_on = ");
  DEBUG_PRINTLN(cntr_on);


}

bool WaterMeter::checkMeter() {
  int curr = analogRead(pin);

  if (needInit) {
    init();
    needInit = false;
  }

  DEBUG_PRINT(pin);
  DEBUG_PRINT("  curr=");
  DEBUG_PRINT(curr);
  DEBUG_PRINT("  cntr_on=");
  DEBUG_PRINT(cntr_on);
  DEBUG_PRINT("  counter=");
  DEBUG_PRINTLN(counter);

  if ((cntr_on) && (curr >= low_min) && (curr <= low_max) && ((millis() - curr_time) >= WAIT_TIME)) {
    cntr_on = false;
    counter++;
    DEBUG_PRINTLN(" +++ 1");
    curr_time = millis();
    return true;
  } else if (((curr < low_min) || (curr > low_max)) && ((millis() - curr_time) >= WAIT_TIME)) {
    cntr_on = true;
  }

  return false;
}

uint16_t WaterMeter::getCounter() {
  return counter;
}

uint16_t WaterMeter::getCounterReset() {
  uint16_t result = counter;
  counter = 0;
  return result;
}

