/* 
* Motor1.cpp
*
* Created: 7/19/2019 9:51:20 PM
* Author: Daniel_M
*/

#include "Arduino.h"
#include "Motor1.h"

#define m_forward 9
#define m_reverse 10


// default constructor
Motor1::Motor1()
{
} //Motor1

// default destructor
Motor1::~Motor1()
{
} //~Motor1

Motor1::Motor1(byte pin1, byte pin2)
{
	_pin1=pin1;//here we store the pin number in private variable _pin
	_pin2=pin2;
}

void Motor1::begin() //this initializes the pin
{
	DDRB |=(1 << _pin1) | (1 << _pin2); //pin as output   pinMode(_pin, OUTPUT); sau definit si in limbaj arduino
}

void Motor1::forward(byte speed1) //this returns the current status
{
	_speed1=255-speed1;
	analogWrite(m_forward, 255);
	analogWrite(m_reverse, _speed1);
}

void Motor1::reverse(byte speed2) //this returns the current status
{
	_speed2=255-speed2;
	analogWrite(m_forward, _speed2);
	analogWrite(m_reverse, 255);
}

void Motor1::braking() //this returns the current status
{
	analogWrite(m_forward, 255);
	analogWrite(m_reverse, 255);
	delay(10);
}