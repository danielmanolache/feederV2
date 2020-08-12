/*
* Led1.h
*
* Created: 10.07.2020 17:06:49
* Author: Daniel-M
*/


#ifndef __LED1_H__
#define __LED1_H__


class Led1
{
	//variables
	public:
	Led1(byte pin);
	void begin();
	void on();
	void off();
	bool status();
	
	protected:
	
	private:
	byte _pin;
	bool _status;

	//functions
	public:
	Led1();
	~Led1();
	protected:
	private:
	Led1( const Led1 &c );
	Led1& operator=( const Led1 &c );

}; //Led1

#endif //__LED1_H__
