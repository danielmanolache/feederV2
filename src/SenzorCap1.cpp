/* 
* SenzorCap1.cpp
*
* Created: 10.07.2020 16:33:06
* Author: Daniel-M
*/

#include "Arduino.h"
#include "../include/SenzorCap1.h"

// default constructor
SenzorCap1::SenzorCap1()
{
} //SenzorCap1

// default destructor
SenzorCap1::~SenzorCap1()
{
} //~SenzorCap1

//class SenzorCap--------------------------------------------------------------

SenzorCap1::SenzorCap1(byte pin)
{
	_pin=pin;//here we store the pin number in private variable _pin
}

void SenzorCap1::begin() //this initializes the pin
{
	DDRB &=~(1 << _pin); //pinMode(_pin, INPUT);
}

bool SenzorCap1::status() //this returns the current status
{
	if (PINB & (1 << _pin))
	{return 1;}
	else
	{return 0;}
	//return _status
}

//sfarsit class SenzorCap--------------------------------------------------------------
