/* 
* EncoderMotor1.cpp
*
* Created: 10.07.2020 18:07:45
* Author: Daniel-M
*/

#include "Arduino.h"
#include "../include/EncoderMotor1.h"

// default constructor
EncoderMotor1::EncoderMotor1()
{
} //EncoderMotor1

// default destructor
EncoderMotor1::~EncoderMotor1()
{
} //~EncoderMotor1

EncoderMotor1::EncoderMotor1(byte pin)
{
	_pin=pin;//here we store the pin number in private variable _pin
}

void EncoderMotor1::begin() //this initializes the pin
{
	DDRC &=~(1 << _pin); //pinMode(_pin, INPUT);
}

int EncoderMotor1::directie() //this returns the current status
{
	if (PINC & (1 << _pin))
	{return 1;}
	else
	{return -1;}
}