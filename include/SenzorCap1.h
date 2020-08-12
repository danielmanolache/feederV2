/*
* SenzorCap1.h
*
* Created: 10.07.2020 16:33:06
* Author: Daniel-M
*/


#ifndef __SENZORCAP1_H__
#define __SENZORCAP1_H__


class SenzorCap1
{
	//variables
	public:
	SenzorCap1(byte pin);
	void begin();
	bool status();
	
	protected:
	
	private:
	byte _pin;
	bool _status;

	//functions
	public:
	SenzorCap1();
	~SenzorCap1();
	protected:
	private:
	SenzorCap1( const SenzorCap1 &c );
	SenzorCap1& operator=( const SenzorCap1 &c );

}; //SenzorCap1

#endif //__SENZORCAP1_H__
