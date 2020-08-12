/* 
* Led1.cpp
*
* Created: 10.07.2020 17:06:49
* Author: Daniel-M
*/

#include "Arduino.h"
#include "../include/Led1.h"

// default constructor
Led1::Led1()
{
} //Led1

// default destructor
Led1::~Led1()
{
} //~Led1

Led1::Led1(byte pin)
{
	_pin=pin;//here we store the pin number in private variable _pin
}
void Led1::begin() //this initializes the pin (PORTD)
{
	DDRD |=(1 << _pin); //pin as output   pinMode(_pin, OUTPUT);
}

void Led1::on() //this turns the led on
{
	PORTD &=~(1 << _pin); //pin low is on      digitalWrite(_pin, LOW);
	//_status=1; //set the status property
}

void Led1::off() //this turns the led off
{
	PORTD |= (1 << _pin);//pin high is off    digitalWrite(_pin, HIGH);
	//_status=0; //set the status property
}

bool Led1::status() //this returns the current status
{
	if (PIND & (1 << _pin))
	{return 0;} //led off for pin on
	else
	{return 1;} //led on for pin off
	//return _status;
}
