/*
* EncoderMotor1.h
*
* Created: 10.07.2020 18:07:45
* Author: Daniel-M
*/


#ifndef __ENCODERMOTOR1_H__
#define __ENCODERMOTOR1_H__


class EncoderMotor1
{
	//variables
	public:
	EncoderMotor1(byte pin);
	void begin();
	int directie();

	protected:

	private:
	byte _pin;
	bool _status;

	//functions
	public:
	EncoderMotor1();
	~EncoderMotor1();
	protected:
	private:
	EncoderMotor1( const EncoderMotor1 &c );
	EncoderMotor1& operator=( const EncoderMotor1 &c );

}; //EncoderMotor1

#endif //__ENCODERMOTOR1_H__
