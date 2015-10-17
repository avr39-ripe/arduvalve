
#define DEBUG 1

#ifdef DEBUG
#define DEBUG_PRINTLN(x)  Serial.println (x)
#endif

#include <OneWire.h>
const uint8_t temp_reads = 5;
uint8_t _temp_counter = 0;
float _temp_accum = 0;
float _mode_curr_temp = 26.07;

//Sensor indexes
enum SensorIndexes {VALVE = 0u, TANK = 1u};

struct temp_sensor
{
  byte addr[8];
  uint8_t counter;
  float accum;
  float value;
};

temp_sensor temp_sensors[] = {{{0x28, 0xF2, 0x78, 0xCD, 0x05, 0x00, 0x00, 0xC0},0,0,26.07},
                              {{0x28, 0xD2, 0xCA, 0xCD, 0x05, 0x00, 0x00, 0xBA},0,0,26.07}};

const byte OW_PIN = 6;
const byte INPUT_PIN = 7;
const byte WARM_PIN = 2;
const byte COLD_PIN = 3;
const byte PUMP_PIN = 4;

#define ALL_OFF 255
byte onSwitch = ALL_OFF;

float set_temp = 28.00;
float temp_delta = 0.5;

uint16_t valve_edge_ms = 7 * 1000; // time valve goes from HOT edge to COLD edge in ms
float tank_temp_delta = 0;

float curr_temp;
float last_temp;
float temp;

uint8_t input_state = 1;
uint8_t sleep_mode = FALSE;

byte therm_interval = 30;
unsigned long p_therm_ms = 0;
unsigned long c_therm_ms = 0;

unsigned int switch_interval = 5;
unsigned long p_switch_ms = 0;
unsigned long c_switch_ms = 0;

unsigned int input_interval = 500; //in ms!!!
unsigned long p_input_ms = 0;
unsigned long c_input_ms = 0;


OneWire ds(OW_PIN);

void setup() {
//#ifdef DEBUG
  Serial.begin(9600);
//#endif

  pinMode(WARM_PIN,OUTPUT);
  pinMode(COLD_PIN,OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(WARM_PIN, HIGH);
  digitalWrite(COLD_PIN, HIGH);
  digitalWrite(PUMP_PIN, HIGH);
  
  pinMode(INPUT_PIN, INPUT);
  
  //DO NOT wait for Delay at startup!
  p_therm_ms = millis() + therm_interval * 1000;
  p_input_ms = millis() + input_interval;
  
}

void loop()
{
  getTemp(TANK);
  
  c_input_ms = millis();
  
  if ( c_input_ms - p_input_ms > input_interval )
  {
     input_state = digitalRead(INPUT_PIN);
     Serial.print("Button state ");Serial.println(input_state);
  }
  
  if ( !input_state && (temp_sensors[TANK].value >= (set_temp - tank_temp_delta))) //BOTH input_state is ON and water in tank is HOT
  {
    sleep_mode = FALSE;
    digitalWrite(PUMP_PIN, LOW); //turn ON pump

    getTemp(VALVE);
    temp = temp_sensors[VALVE].value;
  
    c_therm_ms = millis();
    c_switch_ms = millis();

    if ( c_therm_ms - p_therm_ms > therm_interval*1000 )
    {
      p_therm_ms = c_therm_ms;
      curr_temp = temp;
      if ( onSwitch == ALL_OFF )
      {
        checkThermostat();
        Serial.println("check thermo");
      }
    }

    if ( !(onSwitch == ALL_OFF) )
    {
      if ( c_switch_ms - p_switch_ms > switch_interval*1000 )
      {
        Serial.println(onSwitch);
        digitalWrite(onSwitch, HIGH);
        onSwitch = ALL_OFF;
      }
    }
  }
  else if (!sleep_mode) //Either inpit_state is OFF or COLD water in the tank - just go to sleep
  {
    Serial.println("SLEEP MODE ON");
    sleep_mode = TRUE;
    digitalWrite(PUMP_PIN, HIGH); //turn OFF pump
    
    digitalWrite(WARM_PIN, HIGH); //Enshure that WARM_PIN is OFF!!
    
    digitalWrite(COLD_PIN, LOW); //Turn ON and go to COLD edge
    delay(valve_edge_ms);
    digitalWrite(COLD_PIN, HIGH); //Turn OFF and SLEEEEP :)
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

void getTemp(uint8_t sensor_id)
{
  byte _temp_data[12];
  
  ds.reset();
  ds.select(temp_sensors[sensor_id].addr);
  ds.write(0x4e); // write scratchpad cmd
  ds.write(0xff); // write scratchpad 0
  ds.write(0xff); // write scratchpad 1
  ds.write(0b00111111); // write scratchpad config

  ds.reset();
  ds.select(temp_sensors[sensor_id].addr);
  ds.write(0x44); // start conversion
  
  delay(190);
  
  ds.reset();
//	ds.select(temp_sensors[n].addr);
  ds.select(temp_sensors[sensor_id].addr);
  ds.write(0xBE); // Read Scratchpad

  for (uint8_t i = 0; i < 9; i++)
  {
	_temp_data[i] = ds.read();
  }

  if (OneWire::crc8(_temp_data, 8) != _temp_data[8])
  {
	Serial.println("DS18B20 temp crc error!");
	temp_sensors[sensor_id].counter = 0;
	temp_sensors[sensor_id].accum = 0;
	getTemp(sensor_id);
	return;
}
	float tempRead = ((_temp_data[1] << 8) | _temp_data[0]); //using two's compliment
	if (temp_sensors[sensor_id].counter < temp_reads)
	{
		temp_sensors[sensor_id].counter++;
		temp_sensors[sensor_id].accum += (tempRead / 16);

//		Serial.print("TA "); Serial.println(_temp_accum);
		getTemp(sensor_id);
		return;
	}
	else
	{
		temp_sensors[sensor_id].value = temp_sensors[sensor_id].accum / temp_reads;
		temp_sensors[sensor_id].counter = 0;
		temp_sensors[sensor_id].accum = 0;
//		Serial.print("MT "); Serial.println(_mode_curr_temp);
	}

	Serial.print("Sensor id ");
        Serial.print(sensor_id);
        Serial.print(" ");
        Serial.println(temp_sensors[sensor_id].value);
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
