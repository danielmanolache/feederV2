/*
* Motor1.h
*
* Created: 7/19/2019 9:51:20 PM
* Author: Daniel_M
*/


#ifndef __MOTOR1_H__
#define __MOTOR1_H__

#include "Arduino.h"

class Motor1
{
	//variables
	public:
	Motor1(byte pin1, byte pin2);
	void begin();
	void forward(byte speed1);
	void reverse(byte speed2);
	void braking();
	
	protected:
	
	private:
	byte _pin1;
	byte _pin2;
	byte _speed1;
	byte _speed2;

	//functions
	public:
	Motor1();

	~Motor1();
	protected:
	private:
	Motor1( const Motor1 &c );
	Motor1& operator=( const Motor1 &c );

}; //Motor1

#endif //__MOTOR1_H__




