/* 
* Button1.cpp
*
* Created: 10.07.2020 16:55:43
* Author: Daniel-M
*/

#include "Arduino.h"
#include "../include/Button1.h"

// default constructor
Button1::Button1()
{
} //Button1

// default destructor
Button1::~Button1()
{
} //~Button1

Button1::Button1(byte pin)
{
	_pin=pin;//here we store the pin number in private variable _pin
}

void Button1::begin() //this initializes the pin
{
	DDRD &=~(1 << _pin); //pinMode(_pin, INPUT);
}

bool Button1::status() //this returns the current status
{
	if (PIND & (1 << _pin))
	{return 0;}
	else
	{return 1;}
	//return _status
}