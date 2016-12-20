/* This is the code for an induction cooktop controlled with 
 * external MCU (Arduino) to achieve and maintain target 
 * cooking temperatures of a liquid. 
 */

// ------ Libraries
	// These libraries allow using the temperature sensor
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>
	// A timer library is used to simplify
#include <RBD_Timer.h>
	// A button and LCD library is used
	// to control the user interface.
#include <RBD_Button.h>
#include "rgb_lcd.h"

// ------ CONSTANTS
 
	//Temperature sensor signal wire
#define ONE_WIRE_BUS 7

// Induction constants
	//Wire that sends out PWM for induction cooktop
#define INDUCT_PWM 5
	// A0 - reads voltage on strike pin
#define STRIKE_PIN 0

//Motor pin
	//To transistor pin
#define MOTOR_PIN 6

	//STRIKING VARIABLES
#define RELAY_PIN 4
#define STRIKE_PIN_LENGTH 100
	// Items in array to average
#define STRIKE_SWITCH_DELAY 5
	//how long switch is kept on [ms]
#define STRIKE_SUCCESS_VOLTAGE 600
	//If voltage < 600*.0049, success


// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

//timing
const unsigned long TempInterval = 800;
unsigned long TempStartMillis = 0;

const unsigned long TempPrintInterval = 2000; //how often to print temp
unsigned long TempPrevPrint = 0; //where last print time is stored

bool requestingTemp =false; //If DS18B20 is converting

// ----- Motor control Variables
bool stirrerOn=false;

// ----- Induction cooktop variables
int strikeWire;
int strikeWireReadings[STRIKE_PIN_LENGTH];
	//Put strike wire reading into this array
	//Then average it at end
		//NOTE: int is 2 bytes large

bool striking = false;
bool attemptedStrike = false;
int strikeNdx =0;

unsigned long timeOfLastStrike = 0;
const unsigned long strikeAttemptInterval = 2000;

int inductionPWMpower = 75; //Initial power

// ---- Temperature control
float currentTemp = 0;
float targetTemp = 150; //Initial target
float prevTemp = currentTemp;//in case of error


// Induction control range
#define PWM_MAX 255
#define PWM_MIN 75


// PID CONTROL
	// coefficients
const float Kp = 15;
const float Ki = 0.1;
const float Kd = 5;

float error = 0;
float derivative_error = 0;
float previous_error = 0; //used to calculate derivative error
float integral_error = 0;

bool firstHeatingLoop = true;//so derivative error can be set to 0

	//Time step is in ms, but is converted
	// to seconds in error calculations
unsigned long timestep;

	//System variables
bool cooktopOn = true;


// ------------ States ---------------
/*

	STANDBY
		- When device turns on

		- Set target temperature

		- Switch to running mode

	RUNNING
		- PID Control
		- Data -> Serial out
		- Turn on and off stirrer
		- Return to standby mode
			- turn off induction cooktop

*/

bool state = 0; //0 - standby; 1 - running
// UI Variables

#define MAX_TEMP 200
#define MIN_TEMP 100
//LED Screen
rgb_lcd lcdDisplay;
int displayColorStandby[3] = {125,255,125};
int displayColorHeating[3] = {255,125,125};

//Initialize UI Buttons
RBD::Button startStopButton(8);
RBD::Button increaseTempButton(9);
RBD::Button decreaseTempButton(10);
RBD::Button stirrerToggle(11);

const float tempIncMin = 0.5;
const float tempIncMed = 2;
const float tempIncMax = 5;
	//A longpress is a press longer than 1 sec
RBD::Timer shortpressTimer(250);
RBD::Timer longpressTimer(1000);
RBD::Timer longlongpressTimer(5000);



// --------------- Declare functions

int average_ints(int ints[STRIKE_PIN_LENGTH]);



// ------------------- S E T U P ---------------------
void setup()
{
	// ---- Debugging
	Serial.begin(9600);
	// ----

	// ---- UI intitialization
	lcdDisplay.begin(16,2);
	lcdDisplay.setRGB(
		displayColorStandby[0],
		displayColorStandby[1],
		displayColorStandby[2]
	);
	lcdDisplay.print("MERREL DEVICE");

	// ----- Temperature Sensor Reading Setup
	sensors.begin();
	sensors.setWaitForConversion(false);
	// -----

	// ---- Induction Cooktop control
		//Setup digital output pins
	pinMode(INDUCT_PWM,OUTPUT);
	pinMode(RELAY_PIN,OUTPUT);

		//Initial strike
	striking = true; //Strike right away

	// ---- Motor pin
	pinMode(MOTOR_PIN,OUTPUT);

	//Delay start
	for(int i = 3; i>0; i--)
	{
		Serial.print("Time until start: ");
		Serial.println(i);

		//Set print position for lcdDisplay
		lcdDisplay.setCursor(0,1);
		lcdDisplay.print("Starting in ");
		lcdDisplay.print(i);
		lcdDisplay.print("...");

		delay(1000);
	}
	Serial.println("Initialzing screen for standby state...");

		//Clear display
		lcdDisplay.clear();
			//format lcd screen for standby
			lcdDisplay.clear();
			lcdDisplay.setCursor(0,0);
			lcdDisplay.print("--> ");
			lcdDisplay.print(targetTemp);
			lcdDisplay.setCursor(9,0);
			lcdDisplay.print(" F");

			lcdDisplay.setCursor(0,1);
			lcdDisplay.print("Set temperature.");


	Serial.println("End of \"setup\"");

}


// ------------------- L O O P ------------------------
void loop()
{

	// --- S T A T E   M A C H I N E
	if(startStopButton.onPressed())
	{
		if(state){//SWITCH TO STANDBY
			state = 0;
			Serial.println("State changed to \"0\" - standby");

			lcdDisplay.setRGB(
				displayColorStandby[0],
				displayColorStandby[1],
				displayColorStandby[2]
			);

			//format lcd screen for standby
			lcdDisplay.clear();
			lcdDisplay.setCursor(0,0);
			lcdDisplay.print("--> ");
			lcdDisplay.print(targetTemp);
			lcdDisplay.setCursor(9,0);
			lcdDisplay.print(" F");

			lcdDisplay.setCursor(0,1);
			lcdDisplay.print("Set temperature.");

				//Write pwm - shut induction cooktop off
			analogWrite(INDUCT_PWM,0);
			cooktopOn=false;

			digitalWrite(MOTOR_PIN,0);
			stirrerOn=false;

			Serial.println("Set induction pwm signal to 0");

		}else{// SWITCH TO HEATING
			state = 1;
			Serial.println("State changed to \"1\" - heating");

			// Change LCD Color
			lcdDisplay.setRGB(
				displayColorHeating[0],
				displayColorHeating[1],
				displayColorHeating[2]
			);

			//Set lcd screen for heating
			lcdDisplay.setCursor(0,1);
			lcdDisplay.print("@ ");
			lcdDisplay.print(currentTemp);
			lcdDisplay.print(" F            ");

		}
	}

// --------------------------------------------------------- STANDBY STATE
if(state==0)
{

	// Set Temperature Change
	if(increaseTempButton.onPressed()||decreaseTempButton.onPressed())
	{
		float tempIncSign = 1;
		if(decreaseTempButton.isPressed())
			tempIncSign = -1;

		targetTemp += tempIncMin*tempIncSign;
		longlongpressTimer.restart();
		longpressTimer.restart();
		shortpressTimer.restart();

		//Serial.print("Target temp changed to: ");
		//Serial.println(targetTemp);
	}

	if(increaseTempButton.isPressed()||decreaseTempButton.isPressed())
	{
		if(shortpressTimer.isExpired())
		{
			float tempIncSign = 1;
			if(decreaseTempButton.isPressed())
				tempIncSign = -1;

			if(longlongpressTimer.isExpired()) targetTemp += tempIncMax*tempIncSign;
			else if(longpressTimer.isExpired()) targetTemp += tempIncMed*tempIncSign;
			else targetTemp += tempIncMin*tempIncSign;

			shortpressTimer.restart();

			//Serial.print("Target temp changed to: ");
			//Serial.println(targetTemp);
		}
	}

	//Adjust target temp if out of bounds
	if(targetTemp>MAX_TEMP) targetTemp=MAX_TEMP;
	if(targetTemp<MIN_TEMP) targetTemp=MIN_TEMP;

	//Serial.println("FREEZE DEBUG: LCD update to target temp");

	//Display new targetTemp
	lcdDisplay.setCursor(4,0);
	lcdDisplay.print(targetTemp);
	lcdDisplay.setCursor(9,0);
	lcdDisplay.print(" F");

	//Serial.println("FREEZE DEBUG: End of standby loop");

} // ---------------------------------------- END OF STANDBY STATE


/// ----------------------------------------------- HEATING STATE
if(state==1)
{
	// ----- Temperature Sensor Reading Loop
	if(!requestingTemp) //Always request temperature
	{
		prevTemp = currentTemp;
		// Send the command to get temperatures
		sensors.requestTemperatures();

		requestingTemp=true;
		TempStartMillis = millis();
	}

	//Timestep is whenever we next read current temp
	timestep = millis()-TempStartMillis;

	if(requestingTemp && timestep > TempInterval)
	{

		float currentTemp = sensors.getTempFByIndex(0);
		currentTemp = 1.008*currentTemp-1.742;

		//In case of temperature sensor error
		if(currentTemp<0){currentTemp=prevTemp;}

		//Proportional Control
		error = (targetTemp - currentTemp);

		//Integral Control
			//using clamping anti-windup to prevent
			// unnecessarily large overshoot
		if( ( inductionPWMpower==PWM_MAX || inductionPWMpower<=PWM_MIN)
				&& error*integral_error >= 0)
		{
			integral_error = integral_error; //do nothing
		}else{
			integral_error = integral_error + (float) timestep/1000*error;
		}

		//Derivative Error;
		//For first loop, no derivative error
		if (firstHeatingLoop){
			previous_error=error;
			firstHeatingLoop=false;
		}
		derivative_error = (error-previous_error)/((float)timestep/1000);
		previous_error = error;

		//Compute PWM output
		inductionPWMpower =
			(int) (Kp*error + Ki*integral_error + Kd*derivative_error);

			//Limits to output power
		if(inductionPWMpower>PWM_MAX)
		{
			inductionPWMpower = PWM_MAX;
		}
		else if(inductionPWMpower<0)
		{
			inductionPWMpower = 0;
			cooktopOn = false;
		}
		else if(inductionPWMpower<PWM_MIN)
		{
			inductionPWMpower = PWM_MIN;
		}

		//Write pwm
		analogWrite(INDUCT_PWM,inductionPWMpower);

		//Check if cooktop should be on
		if(!cooktopOn && inductionPWMpower>=PWM_MIN)
		{
			cooktopOn = true;
			striking = true;
		}


			//------------
		//------------------ Print out CSV format data
			//-------------
		if(millis()-TempPrevPrint > TempPrintInterval)
  		{
			//Print time running (ms)
			Serial.print(millis());
			//Print current temp
			Serial.print(",");
			Serial.print(currentTemp);
			//Print target temp
			Serial.print(",");
			Serial.print(targetTemp);
			//Print error
			Serial.print(",");
				Serial.print(error);
			//Print integral error
			Serial.print(",");
			Serial.print(integral_error);
			//Print derivative error
			Serial.print(",");
			Serial.print(derivative_error);
			// PRINT PWM SIGNAL
			Serial.print(",");
			Serial.println(inductionPWMpower);

			//In this state, update lcd screen as well
			lcdDisplay.setCursor(0,1);
			lcdDisplay.print("@ ");
			lcdDisplay.print(currentTemp);
			lcdDisplay.print(" F  ");

  			TempPrevPrint = millis();
  		}

		requestingTemp=false;
	}
	// -----


	// ---- Induction Cooktop control


	//// STRIKING
	if(striking)
	{
		//attempt strike
		if(!attemptedStrike && (millis()-timeOfLastStrike)>strikeAttemptInterval)
		{
			Serial.println("Attempting strike");

			digitalWrite(RELAY_PIN, HIGH);
			delay(STRIKE_SWITCH_DELAY);
			digitalWrite(RELAY_PIN, LOW);

			attemptedStrike = true;
			timeOfLastStrike = millis();

			strikeNdx = 0;
		}

		//collect data
		if(attemptedStrike)
		{
			if(strikeNdx==0) Serial.println("Reading strike pin voltage...");

			if(strikeNdx<STRIKE_PIN_LENGTH)
			{
				strikeWireReadings[strikeNdx]=analogRead(STRIKE_PIN);
				strikeNdx++;
				//Serial.print(strikeNdx);
				//Serial.print(", ");
				//Serial.println(strikeWireReadings[strikeNdx-1]);
			}else{
				//Average data
				int averageVoltageOnStrikeLine = average_ints(strikeWireReadings);

				if(averageVoltageOnStrikeLine < STRIKE_SUCCESS_VOLTAGE)
				{
					striking = false;
					Serial.println("Successful Strike");
					Serial.print("Average strike line voltage: ");
					Serial.println(averageVoltageOnStrikeLine);
				}else{
					Serial.println("Unsuccessful Stike Attempt");
					Serial.print("Average strike line reading: ");
					Serial.println(averageVoltageOnStrikeLine);
				}

				attemptedStrike = false;
			}
		}

	}
	// -----

	//Motor toggle
	if(stirrerToggle.onPressed()){
		if(stirrerOn){
			digitalWrite(MOTOR_PIN, 0);
			stirrerOn=false;
		}else{
			digitalWrite(MOTOR_PIN,1);
			stirrerOn=true;
		}
	}

}//------------------------------------------------ END OF HEATING STATE



}// ------------------- END OF LOOP

	// Function that averages a 100 item array of integers
int average_ints(int ints[STRIKE_PIN_LENGTH])
{
	//Serial.println("Averaging...");
	int average_int;
	//Sum integers (none larger than 1023)
	long sum_ints = 0;

	for(int i=0;i<STRIKE_PIN_LENGTH;i++)
	{
		sum_ints += (long) ints[i];
	}
	average_int = (int) (sum_ints/(long)STRIKE_PIN_LENGTH);

	return average_int;
}

