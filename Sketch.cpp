/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

#include <EEPROM.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
//Beginning of Auto generated function prototypes by Atmel Studio
int trimiteI2C (int i);
void doEncoderMotor();
void doEncoderRoata();
void pas_inainte();
void senzor_cap_activ();
void resetare_encoder();
int pasi_ramasi(); // pasi ramasi de parcursi pana la stop
int pasi_ramasi_reverse();// pasi ramasi de parcursi pana la stop in mod de mers inapoi
int pasi_parcursi(); // pasi care sau parcurs de la utlimul stop calculat
int pasi_parcursi_reverse(); // pasi care sau parcurs de la utlimul stop calculat in mod de mers inapoi
int pid(float viteza_referinta);
int pid_reverse(float viteza_referinta);
void press_buton_red();
void press_buton_black();
void press_both_buttons(); //intrare in modul de setare
// void doEncoderA();
// void doEncoderB();
//void senzor_cap();
//void senzor_roata();
// void press_buton1();
// void press_buton2();
// void press_both_buttons();
// void resetare_encoder();
// void afisare_pozitie();
// void afisare_eroare();
//End of Auto generated function prototypes by Atmel Studio

//output pins
#define m_forward 9
#define m_reverse 10

#define semnal_38k 1
#define I2C_ADDRESS_MAIN_MASTER 1
//#define alimentare_circuit PORTE3

//input pins
#define senzor_roata 3
#define senzor_motor 2
#define senzor2_cap 0
#define senzor_cap_pin 8  //input de la vsop
#define brown_out A2
#define directie_motor_pin A3
volatile int j = 0;

volatile int encoder0Pos;
int encoderstop;
int encoderlast;
int encoderResset;
long lastMilli2;//masurare timp discret
float resset;

//output varibales
volatile bool flag_trimiteI2C = 0;
volatile uint8_t portDhistory = 0xFF;
volatile int directie;
volatile int encoderMotorPos;
bool flag_inainte;
bool flag_roata;
byte pwm_motor;
unsigned long lastMilli;
const int pasul=125;
int pas=125;

//viteza masurata   - de mutat in functia inainte
float i[5];

//class Button--------------------------------------------------------------
class Button
{
	public:
	Button(byte pin);
	void begin();
	bool status();
	private:
	byte _pin;
	bool _status;
};

Button::Button(byte pin)
{
	_pin=pin;//here we store the pin number in private variable _pin
}

void Button::begin() //this initializes the pin
{
	DDRD &=~(1 << _pin); //pinMode(_pin, INPUT);
	
}

bool Button::status() //this returns the current status
{
	
	if (PIND & (1 << _pin))
	{return 0;}
	else
	{return 1;}
	//return _status
}

class Led
{
	public:
	Led(byte pin);
	void begin();
	void on();
	void off();
	bool status();
	private:
	byte _pin;
	bool _status;

};

Led::Led(byte pin)
{
	_pin=pin;//here we store the pin number in private variable _pin
}
void Led::begin() //this initializes the pin (PORTD)
{
	DDRD |=(1 << _pin); //pin as output   pinMode(_pin, OUTPUT);
}

void Led::on() //this turns the led on
{
	PORTD &=~(1 << _pin); //pin low is on      digitalWrite(_pin, LOW);
	//_status=1; //set the status property
}

void Led::off() //this turns the led off
{
	PORTD |= (1 << _pin);//pin high is off    digitalWrite(_pin, HIGH);
	//_status=0; //set the status property
}

bool Led::status() //this returns the current status
{
	if (PIND & (1 << _pin))
	{return 0;} //led off for pin on
	else
	{return 1;} //led on for pin off
	//return _status;
}


//clasa motor------------------------------------
class Motor
{
	public:
	Motor(byte pin1, byte pin2);
	void begin();
	void forward(byte speed1);
	void reverse(byte speed2);
	void braking();
	private:
	byte _pin1;
	byte _pin2;
	byte _speed1;
	byte _speed2;
};

Motor::Motor(byte pin1, byte pin2)
{
	_pin1=pin1;//here we store the pin number in private variable _pin
	_pin2=pin2;
}

void Motor::begin() //this initializes the pin
{
	DDRB |=(1 << _pin1) | (1 << _pin2); //pin as output   pinMode(_pin, OUTPUT); sau definit si in limbaj arduino
}

void Motor::forward(byte speed1) //this returns the current status
{
	_speed1=255-speed1;
	analogWrite(m_forward, 255);
	analogWrite(m_reverse, _speed1);
}

void Motor::reverse(byte speed2) //this returns the current status
{
	_speed2=255-speed2;
	analogWrite(m_forward, _speed2);
	analogWrite(m_reverse, 255);
}

void Motor::braking() //this returns the current status
{
	analogWrite(m_forward, 255);
	analogWrite(m_reverse, 255);
	delay(100);
}
//class SenzorCap--------------------------------------------------------------
class SenzorCap
{
	public:
	SenzorCap(byte pin);
	void begin();
	bool status();
	private:
	byte _pin;
	bool _status;
};

SenzorCap::SenzorCap(byte pin)
{
	_pin=pin;//here we store the pin number in private variable _pin
}

void SenzorCap::begin() //this initializes the pin
{
	DDRB &=~(1 << _pin); //pinMode(_pin, INPUT);
}

bool SenzorCap::status() //this returns the current status
{
	if (PINB & (1 << _pin))
	{return 1;}
	else
	{return 0;}
	//return _status
}

//sfarsit class SenzorCap--------------------------------------------------------------


//class MotorClass--------------------------------------------------------------
class EcoderMotor
{
	public:
	EcoderMotor(byte pin);
	void begin();
	int directie();
	private:
	byte _pin;
	bool _status;
};

EcoderMotor::EcoderMotor(byte pin)
{
	_pin=pin;//here we store the pin number in private variable _pin
}

void EcoderMotor::begin() //this initializes the pin
{
	DDRC &=~(1 << _pin); //pinMode(_pin, INPUT);
}

int EcoderMotor::directie() //this returns the current status
{
	if (PINC & (1 << _pin))
	{return 1;}
	else
	{return -1;}
}

//sfarsit class MotorClass--------------------------------------------------------------

//instantiere outputs
Led led_red(PORTD4);
Led led_green(PORTD5);
Motor mot(PORTB1, PORTB2);

//instantiere inputs
Button buton_red(PORTD6);
Button buton_black(PORTD7);
SenzorCap senzor_cap(PORTB0);
EcoderMotor encoder_directie(PORTC3);

//end class--------------------------------------------------------------

void setup(){
	//setare timere-------------------------------------------------------
	TCCR1B = TCCR1B & B11111000 | B00000001;    //schimbare frecventa PWM - B00000001-31 kHz ; B00000010-3.9 kHz (setare prescaler)
	
	//setare frecventa 38 kHz la Timer 4
	TCCR4A = _BV (COM4A0); // CTC, toggle OC4A on Compare Match
	TCCR4B = _BV (CS40) | _BV(WGM42); // No prescaler, CTC
	OCR4A = 209; // compare A register value (210 * clock speed)
	// = 13.125 nS , so frequency is 1 / (2 * 13.125) = 38095
	//setare timere-------------------------------------------------------

	//output pins
	//creare obiecte----------------------------------
	led_red.begin();
	led_green.begin();
	buton_black.begin();
	buton_red.begin();
	mot.begin();
	//end creare obiecte------------------------------------
	
	//OUTPUT PINS

	DDRE = 0b00001111;
	PORTE = 0b00001111;


	//pinMode(m_forward, OUTPUT); //s-a definit la crearea obiecului mot clasa Motor
	//pinMode(m_reverse, OUTPUT); //s-a definit la crearea obiecului mot clasa Motor
	pinMode(semnal_38k, OUTPUT);
	Wire.begin(I2C_ADDRESS_MAIN_MASTER);
	//input pins
	//pinMode(senzor_cap_pin, INPUT); //input de la iesirea vsop, s-a definit la crearea obiecului senzor_cap clasa SenzorCap
	pinMode(senzor2_cap, INPUT);
	pinMode(senzor_roata, INPUT);
	pinMode(directie_motor_pin, INPUT);
	pinMode(senzor_motor, INPUT);
	pinMode(brown_out, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(senzor_motor), doEncoderMotor, FALLING);
	attachInterrupt(digitalPinToInterrupt(senzor_roata), doEncoderRoata, FALLING);
	//digitalWrite(alimentare_circuit, HIGH);
	led_red.off();
	led_green.on();
	delay(1000);
	//portD=================================================== intreruperi buton_red si Buton _black
	DDRD &= ~((1 << DDD6) | (1 << DDD7)); // Clear the PC2 pin
	// Pd7 (PCINT23 pin)is now inputs

	PORTD |= ((1 << PORTD6) | (1 << PORTD7)); // turn On the Pull-up
	// PC2 is now input with pull-up enabled

	PCICR |= (1 << PCIE2);     // set PCIE2 to enable PCMSK2 scan
	PCMSK2 |= (1 << PCINT22) | (1 << PCINT23);   // set PCINT23 to trigger an interrupt on state change
	//portc======================================================= intreruperi brown_out
	DDRC &= ~(1 << DDC2); // Clear the PC2 pin
	// PC2 (PCINT10 pin)is now inputs

	PORTC |= (1 << PORTC2); // turn On the Pull-up
	// PC2 is now input with pull-up enabled

	PCICR |= (1 << PCIE1);     // set PCIE1 to enable PCMSK1 scan
	PCMSK1 |= (1 << PCINT10);   // set PCINT0 to trigger an interrupt on state change
	
	//portB======================================================== intreruperi senzor_cap vsop
	DDRB &= ~(1 << DDB0); // Clear the PC pin

	PORTB |= (1 << PORTB0); // turn On the Pull-up


	PCICR |= (1 << PCIE0);     // set PCIE to enable PCMSK1 scan
	PCMSK0 |= (1 << PCINT0);   // set PCINT to trigger an interrupt on state change

	sei();                     // turn on interrupt
}


void loop(){
	press_buton_black();
	senzor_cap_activ();
	press_buton_red();   //motor forward
	resetare_encoder();
	press_both_buttons();
	led_green.on();
	led_red.off();
	mot.braking();
	if (flag_trimiteI2C==1){
		trimiteI2C(directie); //trimiteI2C(j);
		flag_trimiteI2C=0;
	}
	delayMicroseconds(50);
}

int trimiteI2C(int i){
	Wire.beginTransmission(8);
	Wire.write(encoderMotorPos%250);	Wire.endTransmission();
	
	return 0;
}
ISR (PCINT0_vect)  //intreruperi senzor_cap vsop
{
	if (senzor_cap.status())// ==1 => senzor activat de capul de la robot
	{
		led_red.on();
		flag_inainte=1;
	}
	else{  //capul de la robot se retrage
		//led_red.off();
	}
}

ISR (PCINT1_vect) //intreruperi brown_out
{
	if(PINC & (1 << PINC2)) // if PINC2 == 1 (HIGH)
	{
		/* LOW to HIGH pin change */
	}
	else
	{
		/* HIGH to LOW pin change */
		led_red.on();
		//EEPROM.write(2, 2);//SCRIE IN MEMORIE
		delay(1000);
	}
}

ISR (PCINT2_vect) //intreruperi buton_red si buton _black
{
	uint8_t changedbits;
	changedbits = PIND ^ portDhistory;
	portDhistory = PIND;
	if(changedbits & (1 << PIND7))
	{
		/* PCINT22 changed */
		if(buton_black.status()==1){
			led_green.on();
		}
		else{

		}
	}
	
	if(changedbits & (1 << PIND6)) //buton rosu
	{
		if(buton_red.status()==1){
			led_red.on();
		}
		else{

		}
	}
}

void doEncoderMotor()
{
	
	directie=encoder_directie.directie();
	cli();
	encoderMotorPos = encoderMotorPos + directie;
	sei();
}

void  doEncoderRoata() {
	flag_roata = 1;
}


void senzor_cap_activ() {//activare motor inainte
	while (senzor_cap.status() == 1) { //senzor cap activ, se aprinde ledul si se asteapta sa se retraga
		led_red.on();
	}
	if (senzor_cap.status() == 0 && flag_inainte == 1) {  // daca cap s-a retras si flag inainte este activ
		led_red.on();
		led_green.off();
		pas=pasul;
		encoderResset = encoderMotorPos - ( encoderMotorPos % pasul);
		encoderstop=encoderResset+pas;
		while (flag_inainte == 1) {                           //inainteaza
			//soft start
			cli();
			lastMilli=millis();
			lastMilli2 = millis();
			int t= millis()-lastMilli; //timpul
			sei();
			const float a = 0.010; //acceleratia
			int timp_acceleratie = 90;
			int pozitie_deceleratie=((timp_acceleratie-4)/2)/(100*a);
			encoderlast=encoderMotorPos;
			float v; //viteza
			float v3; // viteza de deccelerare
			float a3; // acceleratie negativa

			//secventa 1 accelerare
			while(pasi_parcursi() < 50 && t < timp_acceleratie)   //
			{
				cli();
				t = millis()-lastMilli;
				sei();
				v=a*t;
				pid(v);// calculare pid si comanda motor
			}
			//secventa 1 end accelerare

			//secventa 2 constant speed
			cli();
			lastMilli=millis();
			sei();
			v=a*timp_acceleratie;
			while(pasi_ramasi() >= pozitie_deceleratie) //pasi_parcursi() <= pas-pozitie_deceleratie
			{
				pid(v);
			}
			//secventa 2 end constant speed
			
			//secventa 3 - deccelerare
			a3 = -a; // acceleratie negativa
			cli();
			lastMilli=millis();
			sei();
			while(pasi_ramasi() >= 3){//pasi_parcursi() <= pas-3
				cli();
				t = millis()-lastMilli;
				sei();
				v3 = v + a3 * t; //viteza de referinta
				if (v3 < 0.15)
				{
					v3=0.15;
				}
				pid(v3);//pwm_motor=pid(v3);
			}
			//secventa 3 end - deccelerare
			
			//secventa 4 - viteza constanta la oprire
			while(pasi_ramasi() > 0)
			{
				pid(0.15); //viteza-referinta v4 = 0.1
			}
			//secventa 4 end viteza constanta la oprire
			
			mot.braking();
			led_red.off();
			flag_inainte = 0;
			flag_trimiteI2C=1;
			resset=0;  //eroarea cumulata de la pid se reseteaza
		}
	}
}

void press_buton_red(){ // forward
	if(buton_red.status()== 1 && buton_black.status()==0){
		delay(250);
		if (buton_red.status()== 1 && buton_black.status()==0)
		{
			pas=32500;
		}
		else
		{
			pas=pasul;
		}
		led_red.on();
		led_green.off();
		encoderResset = encoderMotorPos - ( encoderMotorPos % pasul);
		encoderstop=encoderResset+pas;
		//soft start
		cli();
		lastMilli=millis();
		lastMilli2 = millis();
		int t= millis()-lastMilli; //timpul
		sei();
		const float a = 0.010; //acceleratia
		int timp_acceleratie = 90;
		int pozitie_deceleratie=((timp_acceleratie-4)/2)/(100*a);
		encoderlast=encoderMotorPos;
		float v; //viteza
		float v3; // viteza de deccelerare
		float a3; // acceleratie negativa

		//secventa 1 accelerare
		while(pasi_parcursi() < 50 && t < timp_acceleratie)   //
		{
			cli();
			t = millis()-lastMilli;
			sei();
			v=a*t;
			pid(v);// calculare pid si comanda motor
		}
		//secventa 1 end accelerare

		//secventa 2 constant speed
		cli();
		lastMilli=millis();
		sei();
		v=a*timp_acceleratie;
		while(pasi_ramasi() >= pozitie_deceleratie) //pasi_parcursi() <= pas-pozitie_deceleratie
		{
			pid(v);
			if (buton_red.status() == 0)
			{
				pas=pasul;
				cli();
				encoderResset = encoderMotorPos - ( encoderMotorPos % pasul);
				sei();
				encoderstop=encoderResset+pas;
				if (pasi_ramasi() < pozitie_deceleratie-1)
				{
					encoderstop=encoderResset+pas+pas;
				}
			}
		}
		//secventa 2 end constant speed
		
		//secventa 3 - deccelerare
		a3 = -a; // acceleratie negativa
		cli();
		lastMilli=millis();
		sei();
		while(pasi_ramasi() >= 3){//pasi_parcursi() <= pas-3
			cli();
			t = millis()-lastMilli;
			sei();
			v3 = v + a3 * t; //viteza de referinta
			if (v3 < 0.15)
			{
				v3=0.15;
			}
			pid(v3);//pwm_motor=pid(v3);
		}
		//secventa 3 end - deccelerare
		
		//secventa 4 - viteza constanta la oprire
		while(pasi_ramasi() > 0)
		{
			pid(0.15); //viteza-referinta v4 = 0.1
		}
		//secventa 4 end viteza constanta la oprire
		mot.braking();
		led_red.off();
		flag_trimiteI2C=1;
		resset=0;  //eroarea cumulata de la pid se reseteaza
	}
}

void press_buton_black(){ // forward
	if(buton_black.status()== 1 && buton_red.status()==0){
		delay(250);
		if (buton_black.status()== 1 && buton_red.status()==0)
		{
			pas=-32500;
		}
		else
		{
			pas=-pasul;
		}
		led_red.on();
		led_green.off();
		encoderResset = encoderMotorPos - ( encoderMotorPos % pasul);
		encoderstop=encoderResset+pas;
		//soft start
		cli();
		lastMilli=millis();
		lastMilli2 = millis();
		int t= millis()-lastMilli; //timpul
		sei();
		const float a = 0.010; //acceleratia
		int timp_acceleratie = 90;
		int pozitie_deceleratie=((timp_acceleratie-4)/2)/(100*a);
		encoderlast=encoderMotorPos;
		float v; //viteza
		float v3; // viteza de deccelerare
		float a3; // acceleratie negativa

		//secventa 1 accelerare
		while(pasi_parcursi_reverse() < 50 && t < timp_acceleratie)   //
		{
			cli();
			t = millis()-lastMilli;
			sei();
			v=a*t;
			pid_reverse(v);// calculare pid si comanda motor
		}
		//secventa 1 end accelerare

		//secventa 2 constant speed
		cli();
		lastMilli=millis();
		sei();
		v=a*timp_acceleratie;
		while(pasi_ramasi_reverse() >= pozitie_deceleratie) //pasi_parcursi() <= pas-pozitie_deceleratie
		{
			pid_reverse(v);
			if (buton_black.status() == 0)
			{
				pas=-pasul;
				cli();
				encoderResset = encoderMotorPos - ( encoderMotorPos % pasul);
				sei();
				encoderstop=encoderResset+pas;
				if (pasi_ramasi_reverse() < pozitie_deceleratie-1)
				{
					encoderstop=encoderResset+pas+pas;
				}
			}
		}
		//secventa 2 end constant speed
		
		//secventa 3 - deccelerare
		a3 = -a; // acceleratie negativa
		cli();
		lastMilli=millis();
		sei();
		while(pasi_ramasi_reverse() >= 3){//pasi_parcursi() <= pas-3
			cli();
			t = millis()-lastMilli;
			sei();
			v3 = v + a3 * t; //viteza de referinta
			if (v3 < 0.15)
			{
				v3=0.15;
			}
			pid_reverse(v3);//pwm_motor=pid(v3);
		}
		//secventa 3 end - deccelerare
		
		//secventa 4 - viteza constanta la oprire
		while(pasi_ramasi_reverse() > 0)
		{
			pid_reverse(0.15); //viteza-referinta v4 = 0.1
		}
		//secventa 4 end viteza constanta la oprire
		mot.braking();
		led_red.off();
		flag_trimiteI2C=1;
		resset=0;  //eroarea cumulata de la pid se reseteaza
	}
}

void resetare_encoder(){
	if (encoderMotorPos > pasul){
		encoderMotorPos = encoderMotorPos % pasul;
				 		Wire.beginTransmission(8);
				 		Wire.write(11);				 		Wire.endTransmission();
	}
	if (encoderMotorPos < -pasul){
		encoderMotorPos = (encoderMotorPos % pasul) ;//+ pasul
				 		Wire.beginTransmission(8);
				 		Wire.write(99);				 		Wire.endTransmission();
	}
}

int pasi_ramasi(){
	return encoderstop-encoderMotorPos;
}

int pasi_ramasi_reverse(){
	return encoderMotorPos-encoderstop;
}

int pasi_parcursi(){
	return pas-(encoderstop-encoderMotorPos);
}

int pasi_parcursi_reverse(){
	return -pas-(encoderMotorPos-encoderstop);
}

int pid(float viteza_referinta){
	static int comanda;
	static int z;
	static const float perioada_esantionare=4;
	cli();
	z = millis()-lastMilli2;//timpul scurs de la utima apelare a functiei
	sei();
	if (z > perioada_esantionare)//daca au trecut .... milisec atunci se masoara
	{
		static float errr;
		static float p; //proportional
		static float _error;//integrator
		static float Viteza_masurata;
		static float previous_error;
		static float derivative;
		lastMilli2 = millis();
		//filtrare viteza masurata
		i[0]=i[1];
		i[1]=i[2];
		cli();
		i[2]=(encoderMotorPos-encoderlast)/perioada_esantionare;//Viteza_masurata=
		encoderlast=encoderMotorPos;
		sei();
		Viteza_masurata=(i[0]+i[1]+i[2])/3;//filtrare viteza masurata
		errr=viteza_referinta-Viteza_masurata;
		derivative = ((errr - previous_error)*0.2) / perioada_esantionare; //derivative = (error - previous_error) / dt
		p= 0.4 * errr; //proportional
		_error=errr*0.07*perioada_esantionare + resset;//integrator
		resset=_error;
		comanda =(p+_error+derivative)*150;//H(r)*EE
		if (comanda > 255)		{			comanda=255;			resset = 255;		}		else if (comanda < 0)		{			comanda=0;			if (resset<0)	   {resset=0;  }
		}
		mot.forward(comanda);
		 		Wire.beginTransmission(8);
		 		Wire.write(pasi_parcursi());				Wire.endTransmission();
	}
	return comanda;
}

int pid_reverse(float viteza_referinta){
	static int comanda;
	static int z;
	static const float perioada_esantionare=4;
	cli();
	z = millis()-lastMilli2;//timpul scurs de la utima apelare a functiei
	sei();
	if (z > perioada_esantionare)//daca au trecut .... milisec atunci se masoara
	{
		static float errr;
		static float p; //proportional
		static float _error;//integrator
		static float Viteza_masurata;
		static float previous_error;
		static float derivative;
		lastMilli2 = millis();
		//filtrare viteza masurata
		i[0]=i[1];
		i[1]=i[2];
		cli();
		i[2]=abs((encoderMotorPos-encoderlast)/perioada_esantionare);//Viteza_masurata=
		encoderlast=encoderMotorPos;
		sei();
		Viteza_masurata=(i[0]+i[1]+i[2])/3;//filtrare viteza masurata
		errr=viteza_referinta-Viteza_masurata;
		derivative = ((errr - previous_error)*0.2) / perioada_esantionare; //derivative = (error - previous_error) / dt
		p= 0.4 * errr; //proportional
		_error=errr*0.07*perioada_esantionare + resset;//integrator
		resset=_error;
		comanda =(p+_error+derivative)*150;//H(r)*EE
		if (comanda > 255)		{			comanda=255;			resset = 255;		}		else if (comanda < 0)		{			comanda=0;			if (resset<0)	   {resset=0;  }
		}
		mot.reverse(comanda);
		  		Wire.beginTransmission(8);
		  		Wire.write(pasi_ramasi_reverse());				Wire.endTransmission();
	}
	return comanda;
}

void press_both_buttons(){//verificare daca se intra in mod de reglare pozitie
	if (buton_red.status() == 1 && buton_black.status() == 1) //ambele butoane apasate
	{
		int n;
		boolean flag_mod_setare_pozitie;
		led_red.on(); //se aprind ambele leduri
		while (buton_red.status() == 1 || buton_black.status() == 1) {
			delay(250);
			n = n + 1;
			if (n >= 6) {  //daca a treut timpul de 1500 ms atunci se sting ambele leduri
				led_green.off();
				led_red.off();
				flag_mod_setare_pozitie = true;
			}
			else{
				flag_mod_setare_pozitie = false;

			}
		}
		while (flag_mod_setare_pozitie == true)// daca flag_mod_setare_pozitie este adevarat, atunci se intra in mod de reglare pozitie
		{

			if (buton_red.status() == 1)//daca buton rosu este apasat atunci se inainteaza o pozitie
			{
				led_red.on();
				int ecoder_plus_1= encoderMotorPos + 1;
				resset=0;
				while (ecoder_plus_1 > encoderMotorPos)
				{
					pid(0.1);
				}
				mot.braking();
				led_red.off();
				delay(25);
			}
			if (buton_black.status() == 1) {
				encoderMotorPos=0;
				led_green.on();
				delay(500);
				flag_mod_setare_pozitie = false;
			}
		}
	}
}