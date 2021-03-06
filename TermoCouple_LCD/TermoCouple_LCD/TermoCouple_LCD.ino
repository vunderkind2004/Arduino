#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
//!!! Change LiquidCrystal_I2C.cpp at line 19 instead of "return 0;" should be: return 1;

// Default connection is using software SPI, but comment and uncomment one of
// the two examples below to switch between software SPI and hardware SPI:

// Example creating a thermocouple instance with software SPI on any three
// digital IO pins.
#define MAXDO   5
#define MAXCS   4
#define MAXCLK  3


unsigned long time;

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// Example creating a thermocouple instance with hardware SPI
// on a given CS pin.
//#define MAXCS   10
//Adafruit_MAX31855 thermocouple(MAXCS);

void setup() {
  Serial.begin(9600);
 
  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc

  lcd.init();                      // initialize the lcd    
  lcd.backlight();
  lcd.setCursor(5, 0);
  lcd.print("TERMOMETR");
    
  Serial.println("time;temperature");
  // wait for MAX chip to stabilize
  delay(500);
}

void loop() {
  // basic readout test, just print the current temp
   //Serial.print("{ \"Tinternal\":");
   double tInternal = thermocouple.readInternal();
   //Serial.print(tInternal);    
   lcd.setCursor(0, 1); //set cursor to 2-nd row
   //lcd.print("Internal T = ");
   //lcd.print(tInternal);
   

   
   lcd.print("T= ");
   //Serial.print(", \"Tcouple\":");
   double c = thermocouple.readCelsius();   
   time = millis();   
   if (isnan(c)) {
     //Serial.println("Something wrong with thermocouple!");
     //Serial.println("error");
     lcd.print("error");    
   } else {    
     lcd.print(c);    
     lcd.setCursor(0, 2); //set cursor to 3 row
     lcd.print("sec=");
     lcd.print(time/1000);
     Serial.print(time/1000);
     Serial.print(";");
     Serial.println(c);
   }
   //Serial.print(", \"time\":");
   
   //Serial.println("}");
   
   //Serial.print("F = ");
   //Serial.println(thermocouple.readFarenheit());
 
   delay(1000);
}
