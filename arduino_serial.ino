/*
 * Create by Nikita Litvinov
 * 3rd
 * 
 */

#include <SoftwareSerial.h>
#include <ArduinoSerialProtocol.h>
#include "Logger.h"
#include "WaterMeter.h"
#include <OneWire.h>
#include <DHT.h>

// The payload that will be sent to the other device

#define WAIT_BEFORE_ANSWER  1
#define WAIT_BEFORE_REPEAT  1
#define LISTENER_INTERVAL   20


// Switches block *********************************************************
#define SWITCH1 2
#define SWITCH2 3
#define SWITCH3 4
#define SWITCH4 5
#define SWITCH5 6

int swSize = 5;
int swPin[5];
bool swState[5];

// Switches block end *****************************************************

// DHT block **************************************************************
#define DHTPIN 11     // what pin we're connected to
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
float dhtTemp;
float dhtHum;

// DHT block end **********************************************************

// DS block ***************************************************************
#define DSPIN   10
OneWire  ds(DSPIN);  // on pin 10 (a 4.7K resistor is necessary)
byte dsAddr[2][8] = {
  {0x28, 0xFF, 0x5E, 0x8C, 0xA1, 0x15, 0x04, 0x9B},
  {0x28, 0xFF, 0xD8, 0x8A, 0xA1, 0x15, 0x04, 0xC5}
};

float dsTemperature[3];
// DS block end ***********************************************************

// WaterMeter block *******************************************************
#define COLD_PIN A0
#define HOT_PIN  A2

WaterMeter cold(COLD_PIN);
WaterMeter hot(HOT_PIN);
// WaterMeter block end ***************************************************

// Serial block ***********************************************************
struct SerialMessage {
  uint8_t cmd;
  uint8_t objType;
  uint8_t objId;

  uint8_t sw;

  int16_t dht_temp;
  int16_t dht_hum;

  int16_t bmp_temp;
  int16_t bmp_press;

  int16_t ds1;
  int16_t ds2;
  //int16_t ds3;
  //int16_t ds4;
  //int16_t ds5;

  int16_t water_hot;
  int16_t water_cold;

  //int16_t ls1;
  //int16_t ls2;
  //int16_t ls3;
  //int16_t ls4;
  //int16_t ls5;

  //int16_t ms1;
  //int16_t ms2;
  //int16_t ms3;


} payload;


ArduinoSerialProtocol protocol(&Serial, (uint8_t*)&payload, sizeof(payload));
uint8_t receiveState;
int cnt = 0;
SerialMessage lastPayload;
bool flash;



// Serial block end *******************************************************


void setup() {
  //Serial.begin(57600);
  Serial.begin(115200);

  swPin[0] = SWITCH1;
  swPin[1] = SWITCH2;
  swPin[2] = SWITCH3;
  swPin[3] = SWITCH4;
  swPin[4] = SWITCH5;

  for (int i = 0; i < swSize; i++)
    pinMode(swPin[i], OUTPUT);

  for (int i = 0; i < swSize; i++)
    setLow(i);
    //swState[i] = LOW;

  payload.cmd = SerialCommand::UNDEFINED;
  payload.objType = SerialCommand::UNDEFINED;
  payload.objId = SerialCommand::UNDEFINED;

  flash = true;
  DEBUG_PRINTLN();
  DEBUG_PRINT("SizeOf payload = ");
  DEBUG_PRINTLN(sizeof(payload));

  dhtTemp = UNDEF;
  dhtHum = UNDEF;

  dsTemperature[0] = UNDEF;
  dsTemperature[1] = UNDEF;
  dsTemperature[2] = UNDEF;

}

void loop() {


  receiveState = protocol.receive();
  //DEBUG_PRINT(receiveState);

  if ((receiveState == ProtocolState::INVALID_CHECKSUM) || (receiveState == ProtocolState::INVALID_SIZE)) {
    delay(WAIT_BEFORE_REPEAT);
    requestRepeatSerialMessage();
  }
  else if (receiveState == ProtocolState::SUCCESS) {
    processSerialMessage();
  }

  if (cnt % 200) {
    cold.checkMeter();
    hot.checkMeter();
  }
  cnt++;
  delay(LISTENER_INTERVAL);

  /*
      DEBUG_PRINT("*** swPin.size=");
      DEBUG_PRINTLN(sizeof swPin);

    for (int i=0; i < swSize; i++) {
      swState[i] = !swState[i];
      digitalWrite((swPin[i]), swState[i]);
      DEBUG_PRINT(i);
      DEBUG_PRINT(" * pin=");
      DEBUG_PRINT(swPin[i]);
      DEBUG_PRINT(" * state=");
      DEBUG_PRINTLN(swState[i]);
      delay(1000);
    }

    listDSSensors();


    swState[1] = HIGH;
    
    setSwOutput(5);

    DEBUG_PRINT("genSw = ");
    DEBUG_PRINTLN(genSw());
    
    delay(5000);
    */

}
// ***********************************************************

// ******************** Serial Functions *********************
void processSerialMessage() {
  blink (13, 50);

  uint8_t cmd = payload.cmd;
  uint8_t objType = payload.objType;
  uint8_t objId = payload.objId;

  DEBUG_PRINTLN();
  DEBUG_PRINT("cmd=");
  DEBUG_PRINT(cmd);
  DEBUG_PRINT(" type=");
  DEBUG_PRINT(objType);
  DEBUG_PRINT(" id=");
  DEBUG_PRINTLN(objId);

  // if needs to repeat send message
  if (cmd == SerialCommand::REPEAT) {
    repeatSerialMessage();
    return;
  }

  DEBUG_PRINTLN ("receiveState = Success");
  if (objType == ObjectType::SWITCH) {
    if ((objId >= ObjectId::SWITCH_1) && (objId <= ObjectId::SWITCH_5)) {
      if (cmd == SerialCommand::SET_LOW) {
        //swState[getSwId(objId)] = LOW;
        //digitalWrite((swPin[getSwId(objId)]), LOW);
        setLow(getSwId(objId));
        delay(WAIT_BEFORE_ANSWER);
        sendSerialMessage (SerialCommand::OK, ObjectType::SWITCH, objId);
      }
      else if (cmd == SerialCommand::SET_HIGH) {
        //swState[getSwId(objId)] = HIGH;
        //digitalWrite((swPin[getSwId(objId)]), HIGH);
        setHigh(getSwId(objId));
        delay(WAIT_BEFORE_ANSWER);
        sendSerialMessage (SerialCommand::OK, ObjectType::SWITCH, objId);
      }
    }

    if ((cmd == SerialCommand::SET_SW) && (objId == ObjectId::ALL)) {
      DEBUG_PRINTLN("Execute setSwOutput()");
      setSwOutput();  
    }
  }



  if (cmd == SerialCommand::COLLECT) {
    if ((objType == ObjectType::SENSORS) || (objType == ObjectType::ALL)) { // Won't check for (payload.objId == ObjectId::ALL)
      dhtRead(&dhtTemp, &dhtHum);
      readDSSensors(dsAddr, dsTemperature, (sizeof dsAddr / sizeof dsAddr[0]));
    }
    else if (objType == ObjectType::DHT) {
      dhtRead(&dhtTemp, &dhtHum);
    }
    else if (objType == ObjectType::DS) {
      readDSSensors(dsAddr, dsTemperature, (sizeof dsAddr / sizeof dsAddr[0]));
    }
  }
  else if (cmd == SerialCommand::GET) {
    if ((objType == ObjectType::SENSORS) || (objType == ObjectType::DHT) || (objType == ObjectType::DS) || (objType == ObjectType::WATER)) {
      cookSensorsPayload();
      sendSerialMessage(SerialCommand::RETURN, ObjectType::SENSORS, ObjectId::ALL);
    }
    else if (objType == ObjectType::SWITCH) {
      cookSwitchesPayload();
      sendSerialMessage(SerialCommand::RETURN, ObjectType::SWITCH, ObjectId::ALL);
    }
    else if (objType == ObjectType::ALL) {
      cookPayload();
      sendSerialMessage(SerialCommand::RETURN, ObjectType::ALL, ObjectId::ALL);
    }
  }
}


uint8_t getSwId(uint8_t objId) {
  if (objId > 0)
    return objId-1;
  else
    return 0;
}

// ***********************************************************

uint8_t sendSerialMessage(uint8_t cmd, uint8_t objType, uint8_t objId)
{

  receiveState = protocol.receive();

  if (receiveState == ProtocolState::SUCCESS)
  {
    processSerialMessage();
  }

  payload.cmd = cmd;
  payload.objType = objType;
  payload.objId = objId;

  lastPayload = payload;
  receiveState = protocol.send();

  return receiveState;
  /*
   while (receiveState != ProtocolState::SUCCESS) {
   delay(1000);
   receiveState = protocol.send();
   }
   */
}

uint8_t repeatSerialMessage() {
  receiveState = protocol.receive();

  if (receiveState == ProtocolState::SUCCESS) {
    processSerialMessage();
  }

  payload = lastPayload;
  receiveState = protocol.send();
  return receiveState;
}

uint8_t requestRepeatSerialMessage() {
  payload.cmd = SerialCommand::REPEAT;
  payload.objType = ObjectType::UNDEFINED;
  payload.objId = ObjectId::UNDEFINED;

  receiveState = protocol.send();

  return receiveState;
}


void receiveSerialMessage() {

  uint8_t receiveState = protocol.receive();

  if ((receiveState == ProtocolState::INVALID_CHECKSUM) || (receiveState == ProtocolState::INVALID_SIZE)) {
    delay(WAIT_BEFORE_REPEAT);
    requestRepeatSerialMessage();
  }
  else if (receiveState == ProtocolState::SUCCESS) {
    processSerialMessage();
  }
}
// ***********************************************************

// ********************   DHT Functions   ********************
void dhtRead(float *temperature, float *humidity) {

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    DEBUG_PRINTLN("Failed to read from DHT sensor!");
    *temperature = UNDEF;
    *humidity = UNDEF;
    return;
  }

  *temperature = t;
  *humidity = h;

  DEBUG_PRINTLN();
  DEBUG_PRINT("Humidity: ");
  DEBUG_PRINT(h);
  DEBUG_PRINT(" %\t");
  DEBUG_PRINT("Temperature: ");
  DEBUG_PRINT(t);
  DEBUG_PRINT(" *C ");
  DEBUG_PRINTLN();
}
// ***********************************************************

// ********************   DS Functions   *********************
float dsRead(byte addr[8]) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  float celsius, fahrenheit;

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  DEBUG_PRINT("  Data = ");
  DEBUG_PRINTHEX(present);
  DEBUG_PRINT(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    DEBUG_PRINTHEX(data[i]);
    DEBUG_PRINT(" ");
  }
  DEBUG_PRINT(" CRC=");
  DEBUG_PRINTHEX(OneWire::crc8(data, 8));
  DEBUG_PRINTLN();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  DEBUG_PRINT("  Temperature = ");
  DEBUG_PRINT(celsius);
  DEBUG_PRINT(" Celsius, ");

  return celsius;
}

void readDSSensors(byte addrs[][8], float *temp, byte sensorsNum) {
  //byte len = sizeof addrs / sizeof addrs[0];
  DEBUG_PRINT("addrs length = ");
  DEBUG_PRINTLN(sensorsNum);

  for (int i = 0; i < sensorsNum; i++)
    temp[i] = dsRead(addrs[i]);
}

void listDSSensors(void) {
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];

  while (ds.search(addr)) {

    DEBUG_PRINT("R=");
    for ( i = 0; i < 8; i++) {
      DEBUG_PRINTHEX(addr[i]);
      DEBUG_PRINT(" ");
    }

    if ( OneWire::crc8( addr, 7) != addr[7]) {
      DEBUG_PRINT("CRC is not valid!\n");
      continue;
    }

    if ( addr[0] == 0x28) {
      DEBUG_PRINT("Device is a DS18B20 family device.\n");
    }

    // The DallasTemperature library can do all this work for you!

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end

    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);         // Read Scratchpad

    DEBUG_PRINT("P=");
    DEBUG_PRINTHEX(present);
    DEBUG_PRINT(" ");
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
      DEBUG_PRINTHEX(data[i]);
      DEBUG_PRINT(" ");
    }
    DEBUG_PRINT(" CRC=");
    DEBUG_PRINTHEX( OneWire::crc8( data, 8));
    DEBUG_PRINTLN();
  }
  DEBUG_PRINT("No more addresses.\n");
  ds.reset_search();
  delay(250);
  return;
}


// ***********************************************************

// ******************** Service Functions ********************

void cookSensorsPayload() {
  payload.dht_temp = (dhtTemp * 100);
  payload.dht_hum = (dhtHum * 100);

  payload.ds1 = (dsTemperature[0] * 100);
  payload.ds2 = (dsTemperature[1] * 100);

  payload.water_hot = hot.getCounterReset();
  payload.water_cold = cold.getCounterReset();

  DEBUG_PRINT("dht_temp=");
  DEBUG_PRINTLN(payload.dht_temp);
  DEBUG_PRINT("dht_hum=");
  DEBUG_PRINTLN(payload.dht_hum);
  DEBUG_PRINT("ds1=");
  DEBUG_PRINTLN(payload.ds1);
  DEBUG_PRINT("ds2=");
  DEBUG_PRINTLN(payload.ds2);
  DEBUG_PRINT("hot_water=");
  DEBUG_PRINTLN(payload.water_hot);
  DEBUG_PRINT("cold_water=");
  DEBUG_PRINTLN(payload.water_cold);
}

void cookSwitchesPayload() {
//  payload.sw1 = getState(0);
//  payload.sw2 = getState(1);
//  payload.sw3 = getState(2);

  payload.sw = (getState(0)) + (getState(1) << 1) + (getState(2) << 2) + (getState(3) << 3) + (getState(4) << 4);

  DEBUG_PRINT("sw=");
  DEBUG_PRINTLN(payload.sw);
  
}

void cookPayload() {
  cookSensorsPayload();
  cookSwitchesPayload();
}



// Relay controles by GND therefore we need to inverse state of Switches

bool getState(uint8_t index) {
  return !swState[index];
}

void setState(uint8_t index, bool state) {
  if (swState[index] == state) {
    swState[index] = !state;
    digitalWrite((swPin[index]), swState[index]);
  }
}

void setHigh(uint8_t index) {
  setState(index, HIGH);
}

void setLow(uint8_t index) {
  setState(index, LOW);
}

void setSwOutput(void) {

  uint8_t sw = payload.sw;
  
  DEBUG_PRINT("sw=");
  DEBUG_PRINTLN(sw);

  DEBUG_PRINT("before set, sw0, 1, 2, 3, 4 =");
  DEBUG_PRINT(getState(0));
  DEBUG_PRINT(", ");
  DEBUG_PRINT(getState(1));
  DEBUG_PRINT(", ");
  DEBUG_PRINT(getState(2));
  DEBUG_PRINT(", ");
  DEBUG_PRINT(getState(3));
  DEBUG_PRINT(", ");
  DEBUG_PRINT(getState(4));
  DEBUG_PRINTLN(); 

  bool state = LOW;

  setState(0, ((sw & 1) == 1)?HIGH:LOW);
  setState(1, (((sw & 2)  >> 1) == 1)?HIGH:LOW);
  setState(2, (((sw & 4)  >> 2) == 1)?HIGH:LOW);
  setState(3, (((sw & 8)  >> 3) == 1)?HIGH:LOW);
  setState(4, (((sw & 16) >> 4) == 1)?HIGH:LOW);

  DEBUG_PRINT("after set sw0, 1, 2, 3, 4 =");
  DEBUG_PRINT(getState(0));
  DEBUG_PRINT(", ");
  DEBUG_PRINT(getState(1));
  DEBUG_PRINT(", ");
  DEBUG_PRINT(getState(2));
  DEBUG_PRINT(", ");
  DEBUG_PRINT(getState(3));
  DEBUG_PRINT(", ");
  DEBUG_PRINT(getState(4));
  DEBUG_PRINTLN(); 
  
}

void blink(int pin, int dt) {

  digitalWrite(pin, HIGH);
  delay(dt);
  digitalWrite(pin, LOW);
  // ***********************************************************
}
