//**********************************************
// **********  Arduino Mega 2560 ***************
//**********************************************

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

#define HEATER  12

//analog pins
#define Tpin A1
#define dTpin A0

int Tread;
int dTread;
int maxRealRead = 1018;

int Tmin = 0;
int Tmax = 600;
int dTmin = 0;
int dTmax = 100;

int Tset;
int dTset;

char* hh;
char* mm;
char* ss;
double c;

bool isHeaterOn = false;

unsigned long time;
unsigned long lastHeaterOnDuration;
unsigned long lastHeaterOffDuration;
unsigned long heaterSwitchedOnTime;
unsigned long heaterSwitchedOffTime;

int led = 13;           // the PWM pin the LED is attached to

//******************** timers ****************************************
//each new timer requires similar three lines of code
// ---------------- Heater -------------------------------------------
unsigned long heaterMiliseconds;
boolean       repeatHeater = true; //repeat this sequence?
boolean       heaterEnabled = true; //is this timer enabled?

//--------------- Load timer -------------------------------------
	
unsigned long loadMiliseconds;
boolean       repeatLoad = true; //repeat this sequence?
boolean       loadEnabled = true; //is this timer enabled?

//********************************************************************


// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// Example creating a thermocouple instance with hardware SPI
// on a given CS pin.
//#define MAXCS   10
//Adafruit_MAX31855 thermocouple(MAXCS);

void setup() {
	pinMode(HEATER, OUTPUT);

	//First clear all three prescaler bits:
	int prescalerVal = 0x07; //create a variable called prescalerVal and set it equal to the binary                                                       number "00000111"
	TCCR0B &= ~prescalerVal; //AND the value in TCCR0B with binary number "11111000"

	//Now set the appropriate prescaler bits:
	prescalerVal = 1; //set prescalerVal equal to binary number "00000001"
	TCCR0B |= prescalerVal; //OR the value in TCCR0B with binary number "00000001"

	SetHeater(true);
	SetHeater(false);

	Serial.begin(9600);

	hh = "00";
	mm = "00";
	ss = "00";

	while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc

	//lcd.begin();                      // initialize the lcd    
	//lcd.backlight();
	

	Serial.println("time;temperature;heaterIsOn;heaterPersentage");
	// wait for MAX chip to stabilize
	delay(500);
}

int ConvertToPhisicalValue(int xMax, int xMin, int xRead)
{
	if (xRead == 0)
		return (int)xMin;

	if (xRead >= maxRealRead)
		return (int)xMax;

	double result = ((((double)xMax - (double)xMin) / maxRealRead)*(xRead));
	return (int)result;
}

void PrintLcdData(double actualTemperature)
{
	//lcd.clear();

	//Actual Temperature 
	lcd.setCursor(0, 0);
	lcd.print("Act: T=");
	if (isnan(actualTemperature)) {
		lcd.print("error");
	}
	else {
		lcd.print(actualTemperature);
	}
	lcd.print("     ");


	//Temperature set
	lcd.setCursor(0, 1);
	lcd.print("SET: T=");
	lcd.print(Tset);
	lcd.print("   ");

	//Delta temperatura set
	lcd.setCursor(12, 1);
	lcd.print("+/- ");
	lcd.print(dTset);
	lcd.print("   ");


	//heater
	lcd.setCursor(0, 2); //set cursor to 3-nd row
	lcd.print("Heater: ");
	if (isHeaterOn)
	{
		lcd.print("ON");
	}
	else
	{
		lcd.print("OFF");
	}
	lcd.print(" ");


	//heater %
	lcd.setCursor(12, 2);
	lcd.print(CalculateHeaterPercentage());
	lcd.print("% ");

	//time
	lcd.setCursor(0, 3); //set cursor to 4 row
	lcd.print("Duration ");
	//lcd.print((int) time/1000);
	lcd.print(TimeToString(time / 1000));

}

void CalculateTime() {
	time = millis();
	//double seconds = time/1000;
	//int h = (int) seconds /3600;
	//seconds = seconds - h * 3600;
	//int m = (int) seconds / 60;
	//int s = (int) seconds - m * 60;  

}

char * TimeToString(unsigned long t)
{
	static char str[12];
	long h = t / 3600;
	t = t % 3600;
	int m = t / 60;
	int s = t % 60;
	sprintf(str, "%04ld:%02d:%02d", h, m, s);
	return str;
}

void ReadPotinciometers()
{
	Tread = analogRead(Tpin);
	dTread = analogRead(dTpin);

	int tmpTset = ConvertToPhisicalValue(Tmax, Tmin, Tread);

	//step = 5 
	Tset = 10 * ((int)(tmpTset / 10));
	if (tmpTset % 10 >= 5)
	{
		Tset += 5;
	}

	dTset = ConvertToPhisicalValue(dTmax, dTmin, dTread);

}

void SetHeater(bool isOn)
{
	if (isOn == isHeaterOn)
	{
		return;
	}

	CalculateTime();

	//turn ON/OFF heater
	if (isOn)
	{
		digitalWrite(HEATER, HIGH);
		heaterSwitchedOnTime = time;

		lastHeaterOffDuration = time - heaterSwitchedOffTime;
	}
	else
	{
		digitalWrite(HEATER, LOW);
		heaterSwitchedOffTime = time;

		lastHeaterOnDuration = time - heaterSwitchedOnTime;
	}


	isHeaterOn = isOn;

}

int CalculateHeaterPercentage()
{

	if (lastHeaterOnDuration + lastHeaterOffDuration == 0)
	{
		return 0;
	}

	int result = (int)(lastHeaterOnDuration * 100 / (lastHeaterOnDuration + lastHeaterOffDuration));
	return result;
}


void DoHeaterJob() {
	ReadPotinciometers();
	//   double tInternal = thermocouple.readInternal();
	c = thermocouple.readCelsius();

	//turn ON/OFF heater
	if (c > Tset + dTset)
	{
		SetHeater(false);
	}

	if (c < Tset - dTset)
	{
		SetHeater(true);
	}

	CalculateTime();

	if (!isnan(c)) {
		//Serial.print(time/1000);
		Serial.print(TimeToString(time / 1000));

		Serial.print(";");
		Serial.print(c);

		Serial.print(";");
		Serial.print(isHeaterOn);

		Serial.print(";");
		Serial.print(CalculateHeaterPercentage());
		Serial.println("%");
	}
	PrintLcdData(c);
}

//Delay time expired function
//lastMillis = time we started, wait = delay in mS, restart = do we start again 
boolean CheckTime(unsigned long &lastMillis, unsigned long wait, boolean restart)
{
	//is the time up for this task?
	if (millis() - lastMillis >= wait)
	{
		//should this start again?
		if (restart)
		{
			//get ready for the next iteration
			lastMillis = millis(); //get ready for the next iteration
		}
		return true;
	}
	return false;

} // END of CheckTime()

bool printMeasurementEnambled = true;
unsigned long timeArr[512];
int I[512];
int U[512];
int E[512];

void ClearArrays()
{
	for (int i = 0; i < 512; i++)
	{
		timeArr[i] = 0;
		I[i] = 0;
		U[i] = 0;		
		E[i] = 0;
	}
}

void DoMeasurement(int pointPosition) 
{	
	//return;
	if (pointPosition == 0)
	{
		c = thermocouple.readCelsius();		
		ClearArrays();
	}
	//I,U,t,E
	timeArr[pointPosition] = millis();
	I[pointPosition] =  analogRead(Tpin);
	U[pointPosition] =  analogRead(dTpin);	
	E[pointPosition] = analogRead(A2);
}

void DoLoad()
{
	int pwmDelay = 30;
	for (int i = 0; i <= 255; i++)
	{
		analogWrite(led, i);
		delay(pwmDelay);
		DoMeasurement(i);
	}
	delay(pwmDelay);
	for (int i = 255; i >= 0; i--)
	{
		analogWrite(led, i);
		delay(pwmDelay);
		DoMeasurement(256 + 255 - i);
	}	
}

void PrintMeasurement() 
{
	if (!printMeasurementEnambled) 
		return;

	Serial.println("i;time;I;U;temp;E");
	for (int i = 0; i < 512; i++)
	{
		Serial.print(i);
		Serial.print(";");

		//Serial.print(TimeToString(timeArr[i] / 1000));
		Serial.print(timeArr[i]);

		Serial.print(";");
		Serial.print(I[i]);

		Serial.print(";");
		Serial.print(U[i]);

		Serial.print(";");
		if (!isnan(c))
		{
			Serial.print(c);
		}
		else {
			Serial.print("null");
		}

		Serial.print(";");
		Serial.println(E[i]);
	}
}

void loop() {

	//CalculateTime();

	//if (heaterEnabled == true && CheckTime(heaterMiliseconds, 1000UL, repeatHeater))
	//{				
	//	DoHeaterJob();
	//	//if you only want this section of code to happen once
	//	//uncomment the next line
	//	//heaterEnabled = false; //disables the above code
	//}

	if (loadEnabled )//&& CheckTime(loadMiliseconds, 30000UL, repeatLoad))
	{
		loadEnabled = false;
		
		//light on
		digitalWrite(HEATER, HIGH);
		delay(30000);

		DoLoad();
		//light off
		digitalWrite(HEATER, LOW);
		PrintMeasurement();
		
		delay(600000);
		loadEnabled = true;
	}

	
}
