/*
* CPPFile1.cpp
*
* Created: 18.07.2020 20:01:15
*  Author: Daniel-M
*/

#include <Arduino.h>
#include "../include/teste.h"


int8_t teste()// adresa si byte
{
	static uint16_t x;
	int8_t _flag_directie;
	if (x<28)
	{
		_flag_directie=1;
	}
	
	else if (x<56)
	{
		_flag_directie=-1;
	}
	else if (x<84)
	{
		_flag_directie=1;
	}
	else
	{
		_flag_directie=0;
	}
	
	if (x >= 255)
	{
		x=0;
	}
	delay(500);
	x++;
	return _flag_directie;
}