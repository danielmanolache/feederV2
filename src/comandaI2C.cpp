/*
 * CPPFile1.cpp
 *
 * Created: 10.07.2020 13:48:55
 *  Author: Daniel-M
 */ 

#include <Arduino.h>
#include <Wire.h>
#include "../include/comandaI2C.h"

void comandai2c(int _i, int _j)// adresa si byte
{
		Wire.beginTransmission(_i);
		Wire.write(_j);
		Wire.endTransmission();
}