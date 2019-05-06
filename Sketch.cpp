/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

#include <EEPROM.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <avr/sleep.h>
//Beginning of Auto generated function prototypes by Atmel Studio
int trimiteI2C (int i);
void doEncoderMotor();
void doEncoderRoata();
void senzor_cap_activ();
void resetare_encoder();
uint16_t pasi_ramasi(); // pasi ramasi de parcursi pana la stop
uint16_t pasi_parcursi(); // pasi care sau parcurs de la utlimul stop calculat
int pid(float viteza_referinta);
int activare_motor();
void press_both_buttons(); //intrare in modul de setare
void receiveEvent(int howMany);
int send_i2c_response();
int comands_handling();
void reset_pid_variables();
int seleep_enable();// going to sleep is voltage BOD detection
uint8_t citire_secventa_butoane(uint8_t a);

//End of Auto generated function prototypes by Atmel Studio

//output pins
#define m_forward 9
#define m_reverse 10

#define semnal_38k 1
//#define alimentare_circuit PORTE3

//input pins
#define senzor_roata 3
#define senzor_motor 2
#define senzor2_cap 0
#define senzor_cap_pin 8  //input de la vsop
#define brown_out A2
#define directie_motor_pin A3

uint16_t encoderstop; //))))))))))))) uint16_t
uint16_t encoderlast;
uint16_t encoderResset;
unsigned long lastMilli2;//masurare timp discret
float previous_error;
float integral;

//output varibales
volatile bool flag_trimiteI2C = 0;
volatile uint8_t portDhistory = 0xFF;
volatile int directie;
volatile uint16_t encoderMotorPos;//))))))))))))))) uint16_t
int8_t flag_inainte;
int8_t flag_low_voltage;
bool flag_roata;
byte pwm_motor;
unsigned long lastMilli;
uint16_t pasul;

uint8_t lungime_pas_mm;
const float acceleratia=0.016;
int pas=125;
unsigned int t;
uint16_t kk;

//viteza masurata   - de mutat in functia inainte
uint8_t ii[100];
float ff[5];


uint32_t SLAVE_ADRESS=850;
uint8_t adress_b1, adress_b2, adress_b3; // slave address b1 - most signficant byte, b2 - last significant byte
uint8_t flag_acknoledge_i2c = 0;
unsigned char command_recived_i2c[] = {0, 0, 0, 0};

uint8_t cod_butoane; //mod setare de la butoane



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
	
	
	// 	adress_b1 = 0b01111000 | SLAVE_ADRESS >> 16; //most signficant byte
	// 	adress_b2 = SLAVE_ADRESS >> 8; //most signficant byte
	// 	adress_b3 = SLAVE_ADRESS;
	// 	EEPROM.write(100, adress_b1);
	// 	EEPROM.write(101, adress_b2);
	// 	EEPROM.write(102, adress_b3	);
	adress_b1 = EEPROM.read(100);
	adress_b2 = EEPROM.read(101);
	adress_b3 = EEPROM.read(102);
	lungime_pas_mm = EEPROM.read(110); //dimensiunea pasului in mm
	pasul = lungime_pas_mm*125/2;
	SLAVE_ADRESS = adress_b1 << 16 | adress_b2 <<8 | adress_b3;// adresa I2C
	encoderMotorPos= EEPROM.read(1);
	
	//OUTPUT PINS

	DDRE = 0b00001111;
	PORTE = 0b00000111; // activare npn PE3 LOW


	//pinMode(m_forward, OUTPUT); //s-a definit la crearea obiecului mot clasa Motor
	//pinMode(m_reverse, OUTPUT); //s-a definit la crearea obiecului mot clasa Motor
	pinMode(semnal_38k, OUTPUT);
	Wire.begin(adress_b1);
	Wire.onReceive(receiveEvent);
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
	
	

	led_green.on();
	led_red.on();
	delay(1000);
	led_red.off();
	//in caz ca a aparut o intrerupere intrerupt lumina la pornire atunci se
	sei();                     // turn on interrupt
	delay(100);
	flag_inainte=0;
	if ((encoderMotorPos % pasul) > 3 )//daca encodermotorposition este mult in afara atunci se atetioneaza
	{
		led_red.on();
	}
	else
	{
		led_red.off();
	}
}


void loop(){
	seleep_enable();
	activare_motor();   //motor forward
	resetare_encoder();
	press_both_buttons();
	mot.braking();
	reset_pid_variables();//resset error, integral and derivative to 0
	// 	dtostrf(encoderMotorPos, 3, 2, k);  //convers the float or integer to a string. (floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, empty array);
	// 	if (flag_trimiteI2C==1){
	// 		trimiteI2C(directie); //trimiteI2C(j);
	// 		flag_trimiteI2C=0;
	// 	}
	
	//////trimite pozitie roata
// 	if (flag_roata==1){
// 		// 		trimiteI2C(directie); //trimiteI2C(j);
// 		for(int i=0;i<kk;i++){
// 			trimiteI2C(ii[i]);
// 			//ii[i];
// 		}
// 		kk=0;
// 		flag_roata=0;
// 	}
	send_i2c_response();
	delayMicroseconds(50);
}

int trimiteI2C(int i){
	Wire.beginTransmission(10);
	Wire.write(i);
	Wire.endTransmission();
	return 0;
}

ISR (PCINT0_vect)  //intreruperi senzor_cap vsop
{
	
	if (senzor_cap.status())// ==1 => senzor activat de capul de la robot
	{
		if (flag_inainte==0)
		{
			delayMicroseconds(500);
			if (senzor_cap.status()){
				led_red.on();
				flag_inainte=1;
			}
		}
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
		flag_low_voltage = 0;
	}
	else
	{
		/* HIGH to LOW pin change */
		flag_low_voltage = 1;
		led_red.off();
		led_green.off();
		mot.braking();
		EEPROM.write(1, (encoderMotorPos % pasul));//SCRIE IN MEMORIE
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
			if (flag_inainte==0)
			{
				
				flag_inainte=-1;
			}
		}
		else{

		}
	}
	
	if(changedbits & (1 << PIND6)) //buton rosu
	{
		if(buton_red.status()==1){
			if (flag_inainte==0)
			{
				flag_inainte=1;
			}
		}
		else{

		}
	}
}

void doEncoderMotor()
{
	if (flag_inainte |=0)
	{
		directie = flag_inainte;
	}
	else
	{
		directie=encoder_directie.directie();
	}
	cli();
	encoderMotorPos = encoderMotorPos + directie;
	sei();
}

void  doEncoderRoata() {
	ii[kk]=encoderMotorPos%pasul;
	kk=kk+1;
	flag_roata = 1;
}

int activare_motor(){ // forward
	if((flag_inainte |= 0 || senzor_cap.status() == 1) && flag_low_voltage == 0){ //))))))))))))))))) if(flag_inainte == 1 && buton_black.status()==0){
		if (buton_red.status()== 1 || buton_black.status()== 1) {			delay(250); 		}
		while (senzor_cap.status() == 1) { //senzor cap activ, se aprinde ledul si se asteapta sa se retraga
			led_red.on();
		}
		if (buton_red.status()== 1 &&  buton_black.status()== 1)
		{
			flag_inainte=0;
		}
		if (buton_red.status()== 1 && buton_black.status()==0 || buton_black.status()== 1 && buton_red.status()==0)
		{
			pas=28500*flag_inainte;
		}
		else
		{
			pas=pasul*flag_inainte;//))))))))))))))))))))))))  pas=pasul*flag_inainte; encoderMotorPos = encoderMotorPos % pasul+32500;
		}
		led_red.on();
		led_green.off();
		//verificare pozitie motor si calculare urmatorul stop
		cli();
		if (encoderMotorPos % pasul > pasul-10)//daca motorul a mers inapoi cativa pasi atunci se calculeaza diferit encoderResset
		{
			encoderResset = encoderMotorPos - ( encoderMotorPos % pasul) + pasul;
		}
		else
		{
			encoderResset = encoderMotorPos - ( encoderMotorPos % pasul);
		}
		sei();
		
		encoderstop=encoderResset+pas;
		

		//soft start
		cli();
		lastMilli=millis();
		lastMilli2 = millis();
		t = millis()-lastMilli;
		sei();
		float a = acceleratia; //acceleratia
		int timp_acceleratie;
		if (pas >125)
		{
			timp_acceleratie = 125;
		}
		else
		{
			timp_acceleratie = 90;
		}
		
		int pozitie_deceleratie;
		cli();
		encoderlast=encoderMotorPos;
		sei();
		float v; //viteza
		float d;
		bool flag_Resset = 1; // flag resetare pozitie oprire la luarea degetului de e buton

		//secventa 1 accelerare
		while(pasi_parcursi() < (pasul/2) && t < timp_acceleratie)   //timp_acceleratie
		{
			v=a*t;
			pid(v);// calculare pid si comanda motor
		}
		//secventa 1 end accelerare

		//secventa 2 constant speed
		cli();
		lastMilli=millis();
		t = millis()-lastMilli;
		sei();
		v=a*timp_acceleratie;
		d=((a*timp_acceleratie)/2)*timp_acceleratie;
		pozitie_deceleratie = d;
		while(pasi_ramasi() >= pozitie_deceleratie) // >=pozitie_deceleratie
		{
			pid(v);
			if (buton_red.status() == 0 && buton_black.status() == 0 && flag_Resset == 1)
			{
				pas=pasul*flag_inainte;//)))))))))))))pasul*flag_inainte
				cli();
				if (encoderMotorPos % pasul > pasul-pozitie_deceleratie)//daca motorul a mers inapoi cativa pasi atunci se calculeaza diferit encoderResset
				{
					encoderResset = encoderMotorPos - ( encoderMotorPos % pasul) + pasul;
				}
				else
				{
					encoderResset = encoderMotorPos - ( encoderMotorPos % pasul);
				}
				sei();
				//daca motorul a trecut de pozitia de decleratie atunci se mai adauga un pas la pozitia de oprire
				encoderstop=encoderResset+pas;
				if (pasi_ramasi() < pozitie_deceleratie-1)
				{
					encoderstop=encoderResset+pas+pas;
				}
				flag_Resset = 0;
			}
		}
		//secventa 2 end constant speed
		

		//----------------
		cli();
		lastMilli=millis();
		t = millis()-lastMilli;
		sei();
		
		while(pasi_ramasi() >= 5){//pasi_parcursi() <= pas-3
			v=sqrt((pasi_ramasi()-3)*2*acceleratia);//viteza de referinta se calculeaza in funcie de distanta ramasa v(d)
			if (v < 0.15)
			{
				v=0.15;
			}
			pid(v);//pwm_motor=pid(v3);
		}
		//secventa 3 end - deccelerare

		v=0.15;
		
		//secventa 4 - viteza constanta la oprire
		while(pasi_ramasi() > 0)
		{
			pid(v); //viteza-referinta v4 = 0.1
		}
		//secventa 4 end viteza constanta la oprire
		mot.braking();
		led_red.off();
		led_green.on();
		flag_trimiteI2C=1;
		previous_error=0;//eroarea cumulata de la pid se reseteaza
		integral=0;//eroarea cumulata de la pid se reseteaza
		ff[0]=0;
		ff[1]=0;
		ff[2]=0;
		delay(20);
		flag_inainte=0;

	}
	return 0;
}

void resetare_encoder(){
	if (encoderMotorPos < 30000  || encoderMotorPos > 35000 ){
		cli();
		encoderMotorPos = (encoderMotorPos % pasul) + 32500 - ( 32500 % pasul);
		sei();
	}
}

uint16_t pasi_ramasi(){
	uint16_t p;
	cli();
	p = encoderstop*flag_inainte-encoderMotorPos*flag_inainte;
	sei();
	return p;
}

uint16_t pasi_parcursi(){
	uint16_t p;
	cli();
	p = pas*flag_inainte-(encoderstop*flag_inainte-encoderMotorPos*flag_inainte);
	sei();
	return p;
}

int pid(float viteza_referinta){
	static int output;
	static unsigned int z;
	static const float dt=3;//perioada_esantionare

	cli();
	t = millis()-lastMilli;
	z = millis()-lastMilli2;//timpul scurs de la utima apelare a functiei
	sei();
	
	if (z > dt)//daca au trecut .... milisec atunci se masoara
	{
		static const float Kp=70;
		static const float Ki=10;
		static const float Kd=40;
		static float Viteza_masurata;
		static float error;
		static float derivative;

		
		cli();
		lastMilli2 = millis();
		sei();
		
		//filtrare viteza masurata
		ff[0]=ff[1];
		ff[1]=ff[2];
		cli();
		ff[2]=(encoderMotorPos*flag_inainte-encoderlast*flag_inainte)/dt;//Viteza_masurata=
		encoderlast=encoderMotorPos;
		sei();
		Viteza_masurata=(ff[0]+ff[1]+ff[2])/3;//filtrare viteza masurata
		error=viteza_referinta-Viteza_masurata;//setpoint - measured_value
		integral = integral + error * dt;
		derivative = ((ff[0]+ff[1]+ff[2])/3 - previous_error) / dt;
		output = Kp * error + Ki * integral + Kd * derivative;
		previous_error = error;
		//_______________
		

		if (output > 255)
		{
			output=255;
			integral = 22;
		}
		else if (output < 0)
		{
			output=0;
			if (integral<0)	   {integral=0;  }
		}
		if (flag_inainte == 1)
		{
			mot.forward(output);
		}
		if (flag_inainte == -1)
		{
			mot.reverse(output);
		}
		
				Wire.beginTransmission(10);
				Wire.write(output);
				Wire.endTransmission();
	}
	return output;
}

void reset_pid_variables(){
	previous_error=0;//eroarea cumulata de la pid se reseteaza
	integral=0;//eroarea cumulata de la pid se reseteaza
	ff[0]=0;
	ff[1]=0;
	ff[2]=0;
}


void press_both_buttons(){//verificare daca se intra in mod de reglare pozitie
	if (buton_red.status() == 1 && buton_black.status() == 1) //ambele butoane apasate
	{
		int n;
		float d;
		boolean flag_mod_setare_pozitie;
		led_red.on(); //se aprind ambele leduri
		while (buton_red.status() == 1 || buton_black.status() == 1) {
			delay(250);
			n = n + 1;
			if (n >= 6) {  //daca a treut timpul de 1500 ms atunci se sting ambele leduri
				led_green.off();
				flag_mod_setare_pozitie = true;
			}
			else{
				flag_mod_setare_pozitie = false;

			}
		}
		
		//ajustare microstep
		while (flag_mod_setare_pozitie == true)// daca flag_mod_setare_pozitie este adevarat, atunci se intra in mod de reglare pozitie
		{
			while (flag_inainte == 1) {
				if (buton_red.status() == 1)//daca buton rosu este apasat atunci se inainteaza o pozitie
				{
					led_red.on();
					cli();
					int ecoder_plus_1= encoderMotorPos + 1;
					sei();
					while (ecoder_plus_1 > encoderMotorPos)
					{
						pid(0.1);
					}
					mot.braking();
					led_red.off();
					//delay(25);
				}
				if (buton_black.status() == 1) {
					cli();
					encoderMotorPos=32500;
					sei();
					led_green.on();
					delay(500);
					flag_mod_setare_pozitie = false;
					flag_inainte=0;
				}
			}
			//cod de la butoane
			if (flag_inainte == -1) {
				cod_butoane = citire_secventa_butoane(cod_butoane);
				switch (cod_butoane) {
					case 1:
					led_red.on();
					delay(1000);
					cod_butoane = citire_secventa_butoane(8);
					if(cod_butoane >= 8){lungime_pas_mm=2;}
					else {lungime_pas_mm = 4+4*cod_butoane;}
					pasul = lungime_pas_mm*125/2;
					EEPROM.write(110, lungime_pas_mm);
					led_red.off();
					flag_mod_setare_pozitie = false;
					break;
					case 2:
					led_red.on();
					delay(500);
					led_red.off();
					//golire feeder:
					flag_mod_setare_pozitie = false;
					break;
					default:
					//iesire
					flag_mod_setare_pozitie = false;
					break;
				}
			}
			
		}
		led_red.off();
	}
}

void receiveEvent(int howMany) {
	int j = 0;
	while (0 < Wire.available()) { // loop through all but the last
		command_recived_i2c[j] = Wire.read(); // receive byte as a character
		j++;
	}
	if (adress_b2 == command_recived_i2c[0] && adress_b3 == command_recived_i2c[1]) {
		comands_handling();
	}
}

int send_i2c_response()
{
	if (flag_acknoledge_i2c  != 0) {
		delay(10);
		Serial.print(command_recived_i2c[2]);
		Serial.println(" ");
		uint8_t response_i2c[5];
		response_i2c[0] = adress_b1;
		response_i2c[1] = adress_b2;
		response_i2c[2] = adress_b3;
		if (flag_acknoledge_i2c  == 1) {
			response_i2c[3] = command_recived_i2c[2];
			response_i2c[4] = command_recived_i2c[3];
			} else {
			response_i2c[3] = 0x4E;//N
			response_i2c[4] = 0x41;//A
		}
		Wire.beginTransmission(10); // transmit to master device #10
		Wire.write(response_i2c, 5);       // sends 4 bytes
		Wire.endTransmission();    // stop transmitting
		flag_acknoledge_i2c  = 0;
	}
	return 0;
}

int comands_handling() {
	uint8_t command, parameter;
	command = command_recived_i2c[2];
	parameter = command_recived_i2c[3];
	int micro_step;
	int steps_forward;
	int steps_back;
	int step_size;
	int ecoder_plus_1;//numar de micropasi de ajustat inainte
	switch (command) {
		case 0x50: //setare marime pas
		lungime_pas_mm = parameter; //dimensiunea pasului in mm
		pasul = lungime_pas_mm*125/2;
		EEPROM.write(110, lungime_pas_mm);
		flag_acknoledge_i2c = 1;
		break;
		
		case 0x4D:      //ajustaare micro pasi
		micro_step = parameter;
		led_red.on();
		cli();
		ecoder_plus_1= encoderMotorPos + micro_step;
		sei();
		while (ecoder_plus_1 > encoderMotorPos)
		{
			pid(0.1);
		}
		mot.braking();
		led_red.off();
		delay(25);
		flag_acknoledge_i2c = 1;
		break;
		
		case 0x46:       //inaintare numar pasi
		steps_forward = parameter;
		flag_acknoledge_i2c = 1;
		break;
		
		case 0x52:       //inapoi numar pasi
		steps_back = parameter;
		//flag step_back= TRUE;
		flag_acknoledge_i2c = 1;
		break;
		
		case 0x45:       //evacuare feeder
		step_size = parameter;
		//flag step_back= TRUE;
		flag_acknoledge_i2c = 1;
		break;
		default:
		// if nothing else matches, do the default
		flag_acknoledge_i2c = 2;
		break;
	}
	return 0;
}

int seleep_enable(){// going to sleep is voltage BOD detection
	cli();
	if ((flag_low_voltage == 1) || !(PINC & (1 << PINC2)) )//  verificare pin de brownout tensiune
	{
		mot.braking();
		led_red.off();
		led_green.off();
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		PCICR &= ~(1 << PCIE2) & ~(1 << PCIE0);//disable interrup pcie2 si pcie1
		sleep_enable();
		sei();
		sleep_cpu();
		sleep_disable();
		//cli();
		//CLKPR &= ~(1<<CLKPS0);
		//CLKPR &= ~(1<<CLKPCE);  // disable change of the clock prescaler
		//sei();
		PCICR |= (1 << PCIE2) | (1 << PCIE0);//enable interrup pcie2 si pcie1
		led_green.on();
		led_red.on();
		delay(250);
		//clear interrupt flags
		// 		EIFR |= (1<<INTF0);
		// 		PCIFR |= (1<<PCIF0) | (1<<PCIF2);
		led_red.off();
		flag_inainte=0;
		flag_low_voltage = 0;
	}
	else{
		led_green.on();
	}
	sei();
	return 0;
}


uint8_t citire_secventa_butoane(uint8_t a) {
	int8_t v = 0;
	int i = 0;
	int8_t j = 2;
	while ( (j > -1) && (i < 1000)) {
		if (flag_inainte == 1) {
			v |= (1 << j);
			flag_inainte = 0;
			j--;
			i = 0;
		}
		if (flag_inainte == -1) {
			v &= ~(1 << j);
			flag_inainte = 0;
			j--;
			i = 0;
		}
		if (j == -1) {
			a = v;
		}
		i++;
		delay(10);
	}
	return a;
}