/*
* myfunction.cpp
*
* Created: 06.07.2020 23:41:21
*  Author: Daniel-M
*/
#include "myfunction.h"
#include "Arduino.h"

void myfunction()
{
	Serial.println("test my function");
}

void test()
{
	int _flag_directie;
	static uint16_t x;
	for (x==0; x++; x<255)
	{
		if (x<32)
		{
			_flag_directie=1;
		} 
		
		else if (x<64)
		{
			_flag_directie=-1;
		}
		else if (x<96)
		{
			_flag_directie=1;
		}
		delay(500);
		
		
	}
};