
#define DEBUG 1

#ifdef DEBUG
#define DEBUG_PRINTLN(x)  Serial.println (x)
#endif

#include <OneWire.h>
const uint8_t temp_reads = 5;
uint8_t _temp_counter = 0;
float _temp_accum = 0;
float _mode_curr_temp = 26.07;

const byte OW_PIN = 6;
const byte WARM_PIN = 2;
const byte COLD_PIN = 3;

#define ALL_OFF 255
byte onSwitch = ALL_OFF;

float set_temp = 28.00;
float temp_delta = 0.5;

float curr_temp;
float last_temp;
float temp;

byte therm_interval = 30;
unsigned long p_therm_ms = 0;
unsigned long c_therm_ms = 0;

unsigned int switch_interval = 5;
unsigned long p_switch_ms = 0;
unsigned long c_switch_ms = 0;



OneWire ds(OW_PIN);

void setup() {
//#ifdef DEBUG
  Serial.begin(9600);
//#endif

  pinMode(WARM_PIN,OUTPUT);
  pinMode(COLD_PIN,OUTPUT);
  digitalWrite(WARM_PIN, HIGH);
  digitalWrite(COLD_PIN, HIGH);

  //DO NOT wait for Delay at startup!
  p_therm_ms = millis() + therm_interval * 1000;
}

void loop() {

//  temp = getTemp();
getTemp();
temp = _mode_curr_temp;

  c_therm_ms = millis();
  c_switch_ms = millis();

  if ( c_therm_ms - p_therm_ms > therm_interval*1000 ) {

    p_therm_ms = c_therm_ms;
    
//    getTemp();
//    temp = _mode_curr_temp;
    
    curr_temp = temp;
    if ( onSwitch == ALL_OFF ){

    	  checkThermostat();
    	  Serial.println("check thermo");
      }
  }

  if ( !(onSwitch == ALL_OFF) ){
    if ( c_switch_ms - p_switch_ms > switch_interval*1000 ) {
    Serial.println(onSwitch);
      digitalWrite(onSwitch, HIGH);
      onSwitch = ALL_OFF;

    }
  }
}

//float getTemp(){
//  //returns the temperature from one DS18S20 in DEG Celsius
//
//  byte data[12];
//  byte addr[8];
//
//  if ( !ds.search(addr)) {
//      //no more sensors on chain, reset search
//      ds.reset_search();
//      return -1000;
//  }
//
//  if ( OneWire::crc8( addr, 7) != addr[7]) {
//      Serial.println("CRC is not valid!");
//      return -1000;
//  }
//
//  if ( addr[0] != 0x10 && addr[0] != 0x28) {
//      Serial.print("Device is not recognized");
//      return -1000;
//  }
//
//  ds.reset();
//  ds.select(addr);
//  ds.write(0x44); // start conversion, with parasite power on at the end
//
//  byte present = ds.reset();
//  ds.select(addr);
//  ds.write(0xBE); // Read Scratchpad
//
//
//  for (int i = 0; i < 9; i++) { // we need 9 bytes
//    data[i] = ds.read();
//  }
//
//  ds.reset_search();
//
//  byte MSB = data[1];
//  byte LSB = data[0];
//
//  float tempRead = ((MSB << 8) | LSB); //using two's compliment
//  float TemperatureSum = tempRead / 16;
//
//  return TemperatureSum;
//
//}

void getTemp()
{
  byte _temp_data[12];
  
  ds.reset();
  ds.skip();
  ds.write(0x4e); // write scratchpad cmd
  ds.write(0xff); // write scratchpad 0
  ds.write(0xff); // write scratchpad 1
  ds.write(0b00111111); // write scratchpad config

  ds.reset();
  ds.skip();
  ds.write(0x44); // start conversion
  
  delay(190);
  
  ds.reset();
//	ds.select(temp_sensors[n].addr);
  ds.skip();
  ds.write(0xBE); // Read Scratchpad

  for (uint8_t i = 0; i < 9; i++)
  {
	_temp_data[i] = ds.read();
  }

  if (OneWire::crc8(_temp_data, 8) != _temp_data[8])
  {
	Serial.println("DS18B20 temp crc error!");
	_temp_counter = 0;
	_temp_accum = 0;
	getTemp();
	return;
}
	float tempRead = ((_temp_data[1] << 8) | _temp_data[0]); //using two's compliment
	if (_temp_counter < temp_reads)
	{
		_temp_counter++;
		_temp_accum += (tempRead / 16);

//		Serial.print("TA "); Serial.println(_temp_accum);
		getTemp();
		return;
	}
	else
	{
		_mode_curr_temp = _temp_accum / temp_reads;
		_temp_counter = 0;
		_temp_accum = 0;
//		Serial.print("MT "); Serial.println(_mode_curr_temp);
	}

	Serial.print("_mode_curr_temp = "); Serial.println(_mode_curr_temp);
//	return _mode_curr_temp;
}

inline boolean above_range( float temp ){
  if ( temp > set_temp + temp_delta ) return true; else return false;
}

inline boolean below_range( float temp ){
  if ( temp < set_temp - temp_delta ) return true; else return false;
}

inline boolean in_range( float temp ){
  if ( set_temp - temp_delta <= temp <= set_temp + temp_delta ) return true; else return false;
}


void checkThermostat() {
 //switch_interval = abs(curr_temp - last_temp);
 if ( above_range(curr_temp) ) { digitalWrite(COLD_PIN, LOW); digitalWrite(WARM_PIN, HIGH); onSwitch = COLD_PIN; p_switch_ms = millis(); return;}
 if ( below_range(curr_temp) ) { digitalWrite(WARM_PIN, LOW); digitalWrite(COLD_PIN, HIGH); onSwitch = WARM_PIN; p_switch_ms = millis(); return;}
 if ( in_range(curr_temp) ) { digitalWrite(WARM_PIN, HIGH); digitalWrite(COLD_PIN, HIGH); onSwitch = ALL_OFF; return;}
 last_temp = curr_temp;
}
