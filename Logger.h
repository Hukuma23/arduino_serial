/* 
 * File:   Logger.h
 * Author: Nikita
 *
 * Created on 18 Август 2015 г., 16:23
 */

#ifndef LOGGER_H
#define	LOGGER_H

//#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINT(x)      Serial.print(x)
    #define DEBUG_PRINTLN(x)    Serial.println(x)
    #define DEBUG_PRINTHEX(x)   Serial.print(x, HEX)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTHEX(x)
#endif

#define UNDEF -127

#endif	/* LOGGER_H */

