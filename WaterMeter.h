/* 
 * File:   WaterMeter.h
 * Author: Nikita
 *
 * Created on 18 Август 2015 г., 16:43
 */
#include "Arduino.h"
#include "Logger.h"

#ifndef WATERMETER_H
#define  WATERMETER_H

#define R1  10.0
#define R2  5.5
#define R3  1.5

#define TOLL 0.08
#define WAIT_TIME 3000

class WaterMeter {

  public:
    WaterMeter(byte pin, float tolerance = TOLL);
    bool checkMeter();
    uint16_t getCounter();
    uint16_t getCounterReset();
    

  private:
    unsigned long curr_time;
    int low_min;
    int low_max;
    bool cntr_on;
    bool needInit = true;
    byte pin;
    uint16_t counter;
    void init();
  
};


#endif  /* WATERMETER_H */

