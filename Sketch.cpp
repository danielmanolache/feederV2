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
int diferenta_pasi();
int pid(float error);
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
int lastMilli2;//masurare timp discret

//output varibales
volatile bool flag_trimiteI2C = 0;
volatile uint8_t portDhistory = 0xFF; 
volatile int directie;
volatile int encoderMotorPos;
bool flag_inainte;
bool flag_roata;
byte pwm_motor;
unsigned long lastMilli;
static float lasterror; ///variabila pt PID
			float Dreferinta; //distanta de parcurs   - de mutat in functia inainte
			float Dmasurat; //distanta masurata   - de mutat in functia inainte
			float corectie;
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
	led_green.off();
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
	led_red.off();
		digitalWrite(m_reverse, LOW);
		digitalWrite(m_forward, LOW);
	while (buton_black.status()==1)
	{
		//analogWrite(m_forward,200);
		mot.reverse(60);
	}
	while (buton_red.status()==1)
	{
		//analogWrite(m_forward,200);
		mot.forward(150);
		if (flag_roata == 1)
		{
// 				Wire.beginTransmission(8);
// 				Wire.write(byte(encoderMotorPos/250));
// 				Wire.endTransmission();
 				Wire.beginTransmission(8);
 				Wire.write(encoderMotorPos%250);				Wire.endTransmission();
	
			flag_roata = 0;
		}
	}	
	senzor_cap_activ();
	//resetare_encoder();
if (flag_trimiteI2C==1){
// 	Wire.beginTransmission(8);
// 	Wire.write(byte(encoderMotorPos/250));
// 	Wire.endTransmission();
// 	Wire.beginTransmission(8);
// 	Wire.write(encoderMotorPos%250);
// 	Wire.endTransmission();
    //trimiteI2C(directie); //trimiteI2C(j);
	flag_trimiteI2C=0;
}


}

int trimiteI2C(int i){
	Wire.beginTransmission(8);
	Wire.write(i);
	Wire.endTransmission();	
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
	    if(PIND & (1 << PIND7)) // if PIND7 == 1 (HIGH)  buton negru
	    {
		    /* LOW to HIGH pin change */
		    led_red.off();
						j = buton_black.status();
						flag_trimiteI2C=1;
	    }
	    else
	    {
		    /* HIGH to LOW pin change */
		    led_red.on();

	    }
    }
	
    if(changedbits & (1 << PIND6))
    {
	    /* PCINT23 changed */
	    if(PIND & (1 << PIND6)) // if PIND6 == 1 (HIGH)  buton rosu
	    {
		    /* LOW to HIGH pin change */
		    led_red.off();
	    }
	    else
	    {
		    /* HIGH to LOW pin change */
		    led_red.on();
		    j = buton_red.status();
			flag_trimiteI2C=1;
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
	const int pasul=125;
	if (senzor_cap.status() == 0 && flag_inainte == 1) {  // daca cap s-a retras si flag inainte este activ
		led_red.on();
		led_green.off();
		encoderResset = encoderMotorPos - ( encoderMotorPos % pasul);
		encoderstop=encoderResset+pasul;
		int encoderlast2;
		while (flag_inainte == 1) {                           //inainteaza
			//soft start
			lastMilli=millis();
			lastMilli2 = millis();
			int  t= millis()-lastMilli; //timpul
			float v; //viteza 
			//float Dreferinta; //distanta de parcurs
			//float Dmasurat; //distanta masurata
			float err;
			
			const float a = 0.003; //acceleratia
			while(t < 150){  //diferenta_pasidiferenta_pasi() > 100
				t= millis()-lastMilli;
				v=a*t;
				Dreferinta=((a*t)/2)*t;//d=(v/2)*t sau d=do+((v0+V(t))/2)*t si v(t)=v0+a*t distanta de prcurs - teoretica
				Dmasurat = pasul-diferenta_pasi();
				err = Dreferinta-Dmasurat;
				corectie = pid(err); //eroare
				pwm_motor=corectie;
				if (pwm_motor >= 255)
				{
					pwm_motor=255;
				}
				mot.forward(pwm_motor);
				delayMicroseconds(50);
			}
			// end soft start

			// constant speed
			int d0=Dreferinta;//d0=33.75
			lastMilli=millis();
			lastMilli2 = millis();
			while(diferenta_pasi() <= 100 && diferenta_pasi() >= 35){
				t= millis()-lastMilli; //timpul
				Dreferinta=d0+v*t;//d=d0+v*t distanta de pracurs
				Dmasurat = pasul-diferenta_pasi();
				err = Dreferinta-Dmasurat;
				pwm_motor=pid(err);				
				mot.forward(pwm_motor);
				delayMicroseconds(50);
			}
			// constant speed
			
			//soft stop
			int decrement;
			d0=Dreferinta;//d0=92.25
			float v0=v;
			lastMilli=millis();	
			lastMilli2 = millis();		
			while(encoderstop > encoderMotorPos){
				t= millis()-lastMilli; //timpul
				Dreferinta=d0+((v0+v0-a*t)/2)*t;//d=d0+v*t distanta de pracurs
				if (Dreferinta > pasul)
				{
					Dreferinta=pasul;
				
				}
// 					 if (millis()-z > 5){ //daca au trecut .... milisec atunci se masoara
// 						 z=millis();
// 						 		Wire.beginTransmission(8);
// 						 		Wire.write(int(Dreferinta));
// 						 		Wire.endTransmission();
// 						 }
				Dmasurat = pasul-diferenta_pasi();
				err = Dreferinta-Dmasurat;
				pwm_motor=pid(err);
				mot.forward(pwm_motor);
				delayMicroseconds(50);

			}
			//end of soft stop
			mot.braking();
			// 			pozitie[i]=encoder0Pos;
			// 			i=i+1;
			flag_inainte = 0;
		}
		lasterror=0;
		delay(100);
		flag_trimiteI2C=1;
	}
}


//=================VECHI------------------------------------------------------------
/*

#define s_motor 2
#define s_roata 3
#define s_cap A0
#define m_forward 9
#define m_reverse 10
#define led1 4
#define led2 5
#define flop 8
#define buton1 6
#define buton2 7
#define ss A5

volatile int encoder0Pos;
int encoderstop;
int encoderlast;
int encoderResset;
int directie=1; // directia de incrementare encoder motor (1 sau -1)
volatile bool flag_m = true;
volatile bool flag_r = true;
bool flag_b;
int flag_r_pos;
volatile bool flag_c = true;// flag cap
bool flag_ciclu = true;
boolean flag_b1 = true;
volatile bool s_roataState;
//////////masurare perioada timp
unsigned int lastMilli_1 = 0;
unsigned int lastMilli_2 = 0;
int m_PWM = 37; // puterea initaiala a motorului prin factor de umplere
int PWM_proc; // putere motorului prin factor de umplere variabil
#define LOOPTIME_1 1000
#define LOOPTIME_2 1000
#define IMPULSELOOP 100
int val;

void setup() {
  TCCR1B = TCCR1B & B11111000 | B00000001;    //schimbare frecventa PWM - B00000001-31 kHz ; B00000010-3.9 kHz
  pinMode(s_motor, INPUT);
  pinMode(s_roata, INPUT);
  pinMode(s_cap, INPUT);
  pinMode(buton1, INPUT); 
  pinMode(buton2, INPUT);
  pinMode(m_forward, OUTPUT);
  pinMode(m_reverse, OUTPUT);
  pinMode(flop, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(ss, OUTPUT);
  pinMode(led2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(s_motor), doEncoderA, FALLING);
  attachInterrupt(digitalPinToInterrupt(s_roata), doEncoderB, CHANGE);
  analogWrite(m_forward, 255);
  analogWrite(m_reverse, 255);
  PWM_proc = 255 - m_PWM;
  digitalWrite(flop, HIGH);
  digitalWrite(led2, HIGH);
  delay(1000);
  digitalWrite(led2, LOW);
  digitalWrite(led1, HIGH);
  digitalWrite(flop, LOW);
  delay(1000);
  digitalWrite(led1, LOW);
}

  void loop() {
    senzor_cap();
    senzor_roata();
    press_buton1();
    press_buton2();
    press_both_buttons();
    resetare_encoder();
    //afisare_pozitie();
    analogWrite(m_forward, 0);
    analogWrite(m_reverse, 0);
    digitalWrite(led1, LOW);
    digitalWrite(led2, HIGH);
    //afisare_eroare();
  }
  void doEncoderA() {
    encoder0Pos = encoder0Pos + directie;
  }

  void doEncoderB() {
    s_roataState = digitalRead(s_roata);
    flag_r_pos = encoder0Pos;
    flag_r = false;
  }


void senzor_cap() {
  while (digitalRead(s_cap) == LOW) {
    digitalWrite(led1, HIGH);
    digitalWrite(flop, HIGH);
    flag_c = false;
  }
  if (digitalRead(s_cap) == HIGH && flag_c == false) {  // daca senzor cap este activ
    directie=1;
    digitalWrite(led1, HIGH);
    digitalWrite(led2, LOW);
    if ((encoder0Pos % 125) < 63){
      encoderResset = encoder0Pos - ( encoder0Pos % 125);
    }
    else if((encoder0Pos % 125) >= 63){
      encoderResset=encoder0Pos - (encoder0Pos % 125)+63; 
    }
    encoderstop=encoderResset+63;
    PWM_proc = 255 - m_PWM;
    while (flag_c == false) {                           //inainteaza
      analogWrite(m_forward, 255);
      analogWrite(m_reverse, PWM_proc);
      if (encoder0Pos >= encoderstop) {
        analogWrite(m_reverse, 255);
        analogWrite(m_forward, 255);
        flag_c = true;
      }
      else if ((encoderstop - encoder0Pos) <= 25 && encoder0Pos > encoderlast) {
        encoderlast = encoder0Pos;
        PWM_proc = 243 - (encoderstop - encoder0Pos);
      }
    }
    delay(100);
    PWM_proc = 255 - m_PWM;
    digitalWrite(flop, LOW);
    flag_c = true;
    Serial.print ("p: ");
    Serial.print (encoder0Pos);
  }
}


  void  senzor_roata() {
    if (flag_r == false) {
      Serial.print  ("activ");
      Serial.println  (flag_r_pos);
      flag_r = true;
    }
  }

  void press_buton1() {
    if(digitalRead(buton1) == LOW && digitalRead(buton2) == HIGH){
      directie=1;
      if(flag_b1 == true){
        delay(1000);
        flag_ciclu=false;
      }
      digitalWrite(led1, HIGH);
      digitalWrite(led2, LOW);
      if ((encoder0Pos % 125) < 63){
        encoderResset = encoder0Pos - ( encoder0Pos % 125);
      }
      else if((encoder0Pos % 125) >= 63){
        encoderResset=encoder0Pos - (encoder0Pos % 125)+63;
      }
      encoderstop=encoderResset+63;
      PWM_proc = 255 - m_PWM;
      while (flag_ciclu == true) {
        analogWrite(m_forward, 255);
        analogWrite(m_reverse, PWM_proc);
        if (encoder0Pos >= encoderstop) {
          analogWrite(m_reverse, 255);
          analogWrite(m_forward, 255);
          flag_ciclu = false;
        }
        else if ((encoderstop - encoder0Pos) <= 25 && encoder0Pos > encoderlast) {
          encoderlast = encoder0Pos;
          PWM_proc = 243 - (encoderstop - encoder0Pos);
        }
      }
      analogWrite(m_forward, 255);
      analogWrite(m_reverse, 255);
      delay(100);
      flag_b1=false;
    }
  }

  void press_buton2() {
    if(digitalRead(buton2) == LOW && digitalRead(buton1) == HIGH){
      directie=-1;
      digitalWrite(led1, HIGH);
      digitalWrite(led2, LOW);
      while (digitalRead(buton2) == LOW && digitalRead(buton1) == HIGH) {
      digitalWrite(led2, HIGH);
      analogWrite(m_reverse, 255);
      analogWrite(m_forward, 205);
    
        if (encoder0Pos != encoderlast) {
        encoderlast = encoder0Pos;
        Serial.print ("p: ");
        Serial.print (encoder0Pos);
        Serial.print (" t: ");
        Serial.println  (millis());
        }
      }
      analogWrite(m_forward, 255);
      analogWrite(m_reverse, 255);
      delay(100);
    }
  }

  void press_both_buttons() { //intrare in modul de setare 
    if (digitalRead(buton2) == LOW && digitalRead(buton1) == LOW) {
    int n=0;
      while (digitalRead(buton2) == LOW && digitalRead(buton1) == LOW && n < 6) {
        analogWrite(m_forward, 0);
        analogWrite(m_reverse, 0);
        delay(250);
        n = n + 1;
        if (n >= 6) {
          digitalWrite(led1, LOW);
          digitalWrite(led2, LOW);
          flag_b = true;
        }
        else{
          flag_b = false; 
        }
      }
      Serial.print ("setting pozition ");// intra in modul de setare
    }
    if (flag_b ==true){ //
          digitalWrite(led1, LOW);
          digitalWrite(led2, LOW);
          delay(1000);
          directie=1;
          boolean flag2=false;
          boolean flag3=false;
          byte i=0;
          encoderlast = encoder0Pos;
          while (flag_b == true) {
            if(digitalRead(buton2) == LOW && flag3 == false && i < 3){
              i=i+1;
              flag3 = true;
              digitalWrite(led2, HIGH);
            }
            if(digitalRead(buton2) == HIGH && flag3 == true && i < 3){
              flag3=false;
              digitalWrite(led2, LOW);
            }
            if (i>=3 && flag2==false){
              digitalWrite(led1, HIGH);
              digitalWrite(led2, HIGH);
              flag2 = true;
              delay(1000);
            } 
            if(digitalRead(buton1) == LOW && flag2 == true) {
              analogWrite(m_forward, 255);
                analogWrite(m_reverse, 0);
                delayMicroseconds(1500);
                analogWrite(m_reverse, 255);
              digitalWrite(led1, LOW);  
              delay(100); 
              digitalWrite(led1, HIGH);
              delay(100);                     
            }       
            if (digitalRead(buton2) == LOW && flag2 == true) {
              encoder0Pos=0;
              digitalWrite(led1, LOW);
              delay(1000);
              flag_b = false;
            }
          }
    }
  }
  
  void resetare_encoder(){
    if (encoder0Pos > 125){
      encoder0Pos = encoder0Pos % 125;
    }
    if (encoder0Pos < 0){
      encoder0Pos = (encoder0Pos % 125) + 125;
    }
    flag_ciclu = true;
    if(digitalRead(buton1) == HIGH){
      flag_b1=true;
    }
  }
  
  void afisare_pozitie() {
  if (encoder0Pos > encoderlast)    {/////
  encoderlast = encoder0Pos;
  Serial.print ("p: ");
  Serial.println (encoder0Pos);
  }
  }


  void afisare_eroare() {
  //return 0;
  }
*/

  void resetare_encoder(){
    if (encoderMotorPos > 125){
      encoderMotorPos = encoderMotorPos % 125;
    }
    if (encoderMotorPos < 0){
      encoderMotorPos = (encoderMotorPos % 125) + 125;
    }
    //flag_ciclu = true;
   // if(digitalRead(buton1) == HIGH){
    //  flag_b1=true;
    //}
  }
  
  int diferenta_pasi(){
	  return encoderstop-encoderMotorPos;
  }
  
 int pid(float error){
	static int comanda;
	float p;
	float _error;
	int z = millis()-lastMilli2;
	 if (z > 2){ //daca au trecut .... milisec atunci se masoara
		lastMilli2 = millis();
		p= 30 * error;
		lasterror;
		_error=error + lasterror;
		lasterror=_error;
		comanda =(p+_error)*0.2*z+0;//(p+_error)*2+25;
		if (comanda > 255)		{			comanda=255;			lasterror = 215;		}		else if (comanda < 0)		{		  comanda=0;		  //lasterror=0;		}		 
		Wire.beginTransmission(8);
		Wire.write(int(Dmasurat));
		Wire.endTransmission();
	}
	  return comanda;
  }