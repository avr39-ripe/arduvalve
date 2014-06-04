
#define DEBUG 1

#ifdef DEBUG
#define DEBUG_PRINTLN(x)  Serial.println (x)
#endif

#define __PROG_TYPES_COMPAT__
#include <EEPROM.h>
#include <OMEEPROM.h>
#include <OneWire.h>
#include "OMMenuMgr.h"
#include <LiquidCrystal.h>

const byte LCD_RS  = 8;
const byte LCD_EN  = 9;
const byte LCD_D4  = 4;
const byte LCD_D5  = 5;
const byte LCD_D6  = 6;
const byte LCD_D7  = 7;

const byte LCD_ROWS = 2;
const byte LCD_COLS = 16;

const byte BUT_PIN = 0;
const byte OW_PIN = 11;
const byte WARM_PIN = 2;
const byte COLD_PIN = 3;

const int BUTSEL_VAL  = 640;
const int BUTFWD_VAL  = 0;
const int BUTREV_VAL  = 412;
const int BUTDEC_VAL  = 258;
const int BUTINC_VAL  = 100;

const byte BUT_THRESH  = 20;

int BUT_MAP[5][2] = {
                         {BUTFWD_VAL, BUTTON_FORWARD},
                         {BUTINC_VAL, BUTTON_INCREASE},
                         {BUTDEC_VAL, BUTTON_DECREASE},
                         {BUTREV_VAL, BUTTON_BACK},
                         {BUTSEL_VAL, BUTTON_SELECT}
                    };

const byte SET_TEMP_ADDR = 4;
const byte TEMP_DELTA_ADDR = 8;
const byte THERM_INTERVAL_ADDR = 12;

byte hideMenu = 1;

#define ALL_OFF 255
byte onSwitch = ALL_OFF;

float set_temp = 26.07;
float temp_delta = 0.5;

float curr_temp;
float last_temp;
float temp;

byte therm_interval = 30;
unsigned long p_therm_ms = 0;
unsigned long c_therm_ms = 0;

unsigned int switch_interval = 10;
unsigned long p_switch_ms = 0;
unsigned long c_switch_ms = 0;


                        //    TYPE            MAX    MIN    TARGET
MENU_VALUE set_temp_value = { TYPE_FLOAT_100, 85,     0 ,     MENU_TARGET(&set_temp), SET_TEMP_ADDR };
MENU_VALUE temp_delta_value = { TYPE_FLOAT_100, 2,     0.2 ,     MENU_TARGET(&temp_delta), TEMP_DELTA_ADDR };
MENU_VALUE therm_interval_value = { TYPE_BYTE, 0,     0,     MENU_TARGET(&therm_interval), THERM_INTERVAL_ADDR };

                    //        LABEL           TYPE        LENGTH    TARGET
MENU_ITEM item_set_tempme = { {"Set Temperature"}, ITEM_VALUE, 0, MENU_TARGET(&set_temp_value) };
MENU_ITEM item_temp_deltame = { {"Set Delta"}, ITEM_VALUE, 0, MENU_TARGET(&temp_delta_value) };
MENU_ITEM item_therm_intervalme = { {"Set Delay"}, ITEM_VALUE, 0, MENU_TARGET(&therm_interval_value) };
MENU_ITEM item_resetme = { {"Reset values"}, ITEM_ACTION, 0, MENU_TARGET(resetValues) };
MENU_ITEM item_exitme = { {"Exit"}, ITEM_ACTION, 0, MENU_TARGET(uiQwkScreen) };

                   //        List of items in menu level
MENU_LIST const root_list[]   = { &item_set_tempme, &item_temp_deltame, &item_therm_intervalme, &item_resetme, &item_exitme };

                  // Root item is always created last, so we can add all other items to it
MENU_ITEM menu_root     = { {"Root"},        ITEM_MENU,   MENU_SIZE(root_list),    MENU_TARGET(&root_list) };

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
OneWire ds(OW_PIN);
OMMenuMgr Menu(&menu_root);

void setup() {
//#ifdef DEBUG
  Serial.begin(9600);
//#endif


  if( OMEEPROM::saved() )
          loadvars();
      else
          savevars();

  lcd.begin(LCD_COLS, LCD_ROWS);

  uiClear();

  Menu.setDrawHandler(uiDraw);
  Menu.setExitHandler(uiClear);
  Menu.setAnalogButtonPin(BUT_PIN, BUT_MAP, BUT_THRESH);

  initScreen();

  pinMode(WARM_PIN,OUTPUT);
  pinMode(COLD_PIN,OUTPUT);
  digitalWrite(WARM_PIN, HIGH);
  digitalWrite(COLD_PIN, HIGH);

  //DO NOT wait for Delay at startup!
  p_therm_ms = millis() + therm_interval * 1000;
}

void loop() {

  temp = getTemp();

  c_therm_ms = millis();
  c_switch_ms = millis();

  if ( c_therm_ms - p_therm_ms > therm_interval*1000 ) {

    p_therm_ms = c_therm_ms;
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

  if( hideMenu ) {
    Menu.enable(false);
    hideMenu = false;
  }

 byte button = Menu.checkInput();

  if( ! Menu.enable() ) {

      if( button == BUTTON_SELECT ) {
          Menu.enable(true);
      }
      else if( button == BUTTON_FORWARD ) {
          // do something

      }
      //default screen

    lcd.setCursor(3,0);
    lcd.print(set_temp);

    lcd.setCursor(11,0);
    lcd.print(temp);

  }

}


void uiDraw(char* p_text, int p_row, int p_col, int len) {

  lcd.setCursor(p_col, p_row);

  for( int i = 0; i < len; i++ ) {
    if( p_text[i] < '!' || p_text[i] > '~' )
      lcd.write(' ');
    else
      lcd.write(p_text[i]);
  }

}


void uiClear() {

  lcd.clear();
  initScreen();
  Menu.enable(false);

}


void uiQwkScreen() {

  lcd.clear();
  initScreen();
  Menu.enable(false);

}

void initScreen() {

  lcd.setCursor(0,0);
  lcd.print("ST");
  lcd.setCursor(8,0);
  lcd.print("CR");

}

float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;

}

void loadvars() {

    using namespace OMEEPROM;

    read(SET_TEMP_ADDR, set_temp);
    read(TEMP_DELTA_ADDR, temp_delta);
    read(THERM_INTERVAL_ADDR, therm_interval);
}

void savevars() {

    using namespace OMEEPROM;

    write(SET_TEMP_ADDR, set_temp);
    write(TEMP_DELTA_ADDR, temp_delta);
    write(THERM_INTERVAL_ADDR, therm_interval);

}

void resetValues() {

    set_temp = 26.07;
    temp_delta = 0.5;
    therm_interval = 30;

    savevars();
    uiClear();
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
