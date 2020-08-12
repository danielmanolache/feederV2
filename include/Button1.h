/*
* Button1.h
*
* Created: 10.07.2020 16:55:43
* Author: Daniel-M
*/


#ifndef __BUTTON1_H__
#define __BUTTON1_H__


class Button1
{
	//variables
	public:
	Button1(byte pin);
	void begin();
	bool status();
	
	protected:
	
	private:
	byte _pin;
	bool _status;

	//functions
	public:
	Button1();
	~Button1();
	protected:
	private:
	Button1( const Button1 &c );
	Button1& operator=( const Button1 &c );

}; //Button1

#endif //__BUTTON1_H__
