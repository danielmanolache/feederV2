/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
#include "Motor1.h"


/*End of auto generated code by Atmel studio */

#include <EEPROM.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <avr/sleep.h>
//#include "myfunction.h" ++
#include "include/comandaI2C.h"
#include "include/SenzorCap1.h"
#include "include/Button1.h"
#include "include/Led1.h"
#include "include/EncoderMotor1.h"
#include "include/teste.h"

//void myfunction();
//Beginning of Auto generated function prototypes by Atmel Studio
int trimiteI2C (int i);
void doEncoderMotor();
void doEncoderRoata();
void senzor_cap_activ();
uint16_t pasi_ramasi(); // pasi ramasi de parcursi pana la stop
uint16_t pasi_parcursi(); // pasi care sau parcurs de la utlimul stop calculat
int pid(float viteza_referinta);
int activare_motor();
void press_both_buttons(); //intrare in modul de setare
void receiveEvent(int howMany);
int send_i2c_response();
int comands_handling();
void reset_pid_variables();
int sleep_enablef();// going to sleep is voltage BOD detection
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

uint16_t encoderlast;
unsigned long lastMilli2;//masurare timp discret
float previous_error;
float integral;

//output varibales
volatile bool flag_trimiteI2C = 0;
volatile uint8_t portDhistory = 0xFF;
volatile uint16_t encoderMotorPos;//)))))))))))))))
volatile int8_t flag_directie;
int8_t flag_low_voltage;
bool flag_roata;
byte pwm_motor;
unsigned long lastMilli;
uint16_t pasul;
volatile bool flag_power;
uint8_t lungime_pas_mm;
const float acceleratia=0.016;
uint8_t cod_butoane; //mod setare de la butoane
unsigned int t;
uint16_t kk;
uint8_t ii[100];
uint8_t ff[3];
volatile uint8_t pulse_duration;

uint32_t SLAVE_ADRESS=10;
uint8_t adress_b1, adress_b2, adress_b3; // slave address b1 - most signficant byte, b2 - last significant byte
uint8_t flag_acknoledge_i2c = 0;
unsigned char command_recived_i2c[] = {0, 0, 0, 0};
	


//creare obiecte----------------------------------

//instantiere outputs
Led1 led_red(PORTD4);
Led1 led_green(PORTD5);
Motor1 mot(PORTB1, PORTB2);

//instantiere inputs
Button1 buton_red(PORTD6);
Button1 buton_black(PORTD7);
SenzorCap1 senzor_cap(PORTB0);
EncoderMotor1 encoder_motor(PORTC3);


void setup(){
	//setare timere-------------------------------------------------------
	TCCR1B = TCCR1B & B11111000 | B00000001;    //schimbare frecventa PWM comanda motor - B00000001-31 kHz ; B00000010-3.9 kHz (setare prescaler)
	
	//setare frecventa 38 kHz la Timer 4
	TCCR4A = _BV (COM4A0); // CTC, toggle OC4A on Compare Match
	TCCR4B = _BV (CS40) | _BV(WGM42); // No prescaler, CTC
	OCR4A = 209; // compare A register value (210 * clock speed)
	// = 13.125 nS , so frequency is 1 / (2 * 13.125) = 38095
	//setare timere-------------------------------------------------------

	//output pins
	//instantiere obiecte----------------------------------
	led_red.begin();
	led_green.begin();
	buton_black.begin();
	buton_red.begin();
	mot.begin();
	
	
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
	encoderMotorPos = (EEPROM.read(1) + (EEPROM.read(0) << 8)) % pasul;

	//output pins

	DDRE = 0b00001111;
	PORTE = 0b00000111; // activare npn PE3 LOW
	
	// output unused pins - PULLUP
	// 	DDRC |= (1<<DDRC0) | (1<<DDRC1); //	0b00000011;
	// 	PORTC |= (1<<PORTC0) | (1<<PORTC1);
	// 	DDRB |= (1<<DDRB3) | (1<<DDRB4) | (1<<DDRB5);
	// 	PORTB |= (1<<PORTB3) | (1<<PORTB4) | (1<<PORTB5);

	pinMode(semnal_38k, OUTPUT);
	Wire.begin(adress_b1);
	Wire.onReceive(receiveEvent);
	//input pins
	pinMode(senzor2_cap, INPUT_PULLUP);
	pinMode(senzor_roata, INPUT);
	pinMode(directie_motor_pin, INPUT);
	pinMode(senzor_motor, INPUT);
	pinMode(brown_out, INPUT_PULLUP);
	DDRC &=~(1 << DDRC1); //pinMode(_pin, INPUT);
	attachInterrupt(digitalPinToInterrupt(senzor_motor), doEncoderMotor, FALLING);
	attachInterrupt(digitalPinToInterrupt(senzor_roata), doEncoderRoata, FALLING);
	//digitalWrite(alimentare_circuit, HIGH);

	//portD=================================================== intreruperi buton_red si Buton _black
	DDRD &= ~((1 << DDD6) | (1 << DDD7)); // Clear the PC2 pin
	// PD (PCINT23 pin)is now inputs

	PORTD |= ((1 << PORTD6) | (1 << PORTD7)); // turn On the Pull-up
	// PD is now input with pull-up enabled

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
	//timer2
	////////////////////////////
	//PRR0 |=(1 << PRTIM2);
	TCCR2B |= (1 << CS02) | (1 << CS01) | (1 << CS00); //set prescaler to 1024
	TCCR2B &= ~(1 << WGM22); //NORMAL MODE OPERATION
	TCCR2A &= ~(1 << WGM20) & ~(1 << WGM21); //NORMAL MODE OPERATION
	TIMSK2 |= (1 << TOIE2); //enable T2 OVF interrupt

	//////////////////////
	//ceva comment pentru git
	delay(1000);
	led_red.off();
	//in caz ca a aparut o intrerupere intrerupt lumina la pornire atunci se
	sei();                     // turn on interrupt
	delay(100);
	flag_directie=0;
	if ((encoderMotorPos % pasul) <= 3 || (encoderMotorPos % pasul) >= (pasul-3) )//daca encodermotorposition este mult in afara atunci se atetioneaza
	{
		led_red.off();
	}
	else
	{
		led_red.on();
	}
	Wire.beginTransmission(10);
	Wire.write("start");
	Wire.endTransmission();
}


void loop(){
	TIMSK2 |= (1 << TOIE2); //enable T2 OVF interrupt
	activare_motor();   //motor forward
	press_both_buttons();
	TIMSK2 &= ~(1 << TOIE2); //disable TOVF interrupt
	reset_pid_variables();//resset error, integral and derivative to 0
	
	send_i2c_response();
	sleep_enablef();
	//teste begin--------------------------------
	trimiteI2C(1);
	//flag_directie = teste();
	//teste end--------------------------------
	delayMicroseconds(50);
}

int trimiteI2C(int i){
	if (flag_trimiteI2C == 1)
	// 	dtostrf(encoderMotorPos, 3, 2, k);  //convers the float or integer to a string. (floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, empty array);
	{
		for(int j=0;j<kk;j++){
			Wire.beginTransmission(10);
			Wire.write(ii[j]);
			Wire.endTransmission();
		}
		kk=0;
		flag_trimiteI2C=0;
	}
	return 0;
}

ISR (PCINT0_vect)  //intreruperi senzor_cap vsop
{
	
	if (senzor_cap.status())// ==1 => senzor activat de capul de la robot
	{
		if (flag_directie==0)
		{
			delayMicroseconds(500);
			if (senzor_cap.status()){
				led_red.on();
				flag_directie=1;
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
		EEPROM.write(0, ((encoderMotorPos % pasul)>>8));//SCRIE IN MEMORIE
		flag_directie=0;
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
			if (flag_directie==0)
			{
				flag_directie=-1;
			}
		}
		else{

		}
	}
	
	if(changedbits & (1 << PIND6)) //buton rosu
	{
		if(buton_red.status()==1){
			if (flag_directie==0)
			{
				flag_directie=1;
			}
		}
		else{

		}
	}
}


void doEncoderMotor()
{
	ff[0]=ff[1];
	ff[1]=ff[2];
	ff[2]=TCNT2;
	TCNT2 = 0;
	if(flag_directie == 1) //incrementare
	{
		encoderMotorPos++;
	}
	else if (flag_directie == -1) //decrementare
	{
		encoderMotorPos--;
	}
	else
	{
		encoderMotorPos=encoderMotorPos+encoder_motor.directie();
	}

	if (encoderMotorPos == pasul+1) // resetare encoder
	{
		encoderMotorPos=1;
	}
	
	if (encoderMotorPos == pasul/2)
	{
		if (buton_red.status()== 0 && buton_black.status()== 0 )
		{
			flag_power=0;
		}
	}
	
	if ((flag_power == 0) && (encoderMotorPos == 1))
	{
		mot.braking();
		flag_directie=0;
		led_red.off();
		led_green.on();
		
	}

	if (encoderMotorPos == 0) // resetare encoder
	{
		encoderMotorPos=pasul;
	}

}

void  doEncoderRoata() {
	if(encoder_motor.directie()==1)
	{
				ii[kk]=encoderMotorPos;
				kk++;
				if(kk==99){kk=0;}
		 		flag_roata = 1;
 				flag_trimiteI2C=1;
	}
	//comandai2c(10, int(encoderMotorPos));//begin transmission, send (adress, int), end transmission
}

int activare_motor(){ // forward
	if((flag_directie |= 0 || senzor_cap.status() == 1) && flag_low_voltage == 0){ //))))))))))))))))) if(flag_directie == 1 && buton_black.status()==0){
		if (buton_red.status()== 1 || buton_black.status()== 1) {			delay(250); 		}
		while (senzor_cap.status() == 1) { //senzor cap activ, se aprinde ledul si se asteapta sa se retraga
			led_red.on();
		}
		flag_power=1;
		if (buton_red.status()== 1 &&  buton_black.status()== 1)
		{
			flag_directie=0;
			flag_power=0;
		}
		led_red.on();
		led_green.off();
		//verificare pozitie motor si calculare urmatorul stop
		

		//soft start
		cli();
		lastMilli=millis();
		lastMilli2 = millis();
		t = millis()-lastMilli;
		sei();
		float a = acceleratia; //acceleratia
		int timp_acceleratie;
		if (pasul >125)
		{
			timp_acceleratie = 145;
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
		float d; //distanta

		//secventa 1 accelerare
		while((t < timp_acceleratie) && (flag_low_voltage ==0) )   //timp_acceleratie
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
		pozitie_deceleratie = 50;
		while(((pasi_ramasi() >= pozitie_deceleratie) || (flag_power == 1)) && (flag_low_voltage == 0) ) // >=pozitie_deceleratie
		{
			pid(v);
		}
		//secventa 2 end constant speed
		//secventa 3 deccelerare
		cli();
		lastMilli=millis();
		t = millis()-lastMilli;
		sei();
		while(pasi_ramasi() >= 5){//
			v=sqrt((pasi_ramasi()-3)*2*acceleratia);//viteza de referinta se calculeaza in funcie de distanta ramasa v(d)
			if (v < 0.1)
			{
				v=0.1;
			}
			pid(v);//pwm_motor=pid(v3);
		}
		//secventa 3 end - deccelerare

		v=0.1;

		//secventa 4 - viteza constanta la oprire

		do
		{
			pid(v); //viteza-referinta v4 = 0.1
		} while ((flag_directie |= 0) && (flag_low_voltage == 0));
		//secventa 4 end viteza constanta la oprire
		flag_trimiteI2C=1;
		previous_error=0;//eroarea cumulata de la pid se reseteaza
		integral=0;//eroarea cumulata de la pid se reseteaza
		ff[0]=0;
		ff[1]=0;
		ff[2]=0;
// 		 				Wire.beginTransmission(10);
// 						Wire.write(encoderMotorPos);
// 		 				Wire.endTransmission();
	}
	return 0;
}

uint16_t pasi_ramasi(){
	uint16_t p;
	if (flag_directie == 1)
	{
		cli();
		p=(pasul-encoderMotorPos)*flag_directie;
		sei();
		return p;
	}
	else
	{
		cli();
		p=encoderMotorPos*(-flag_directie);
		sei();
		return p;
	}
}

uint16_t pasi_parcursi(){
	uint16_t p;
	if (flag_directie ==1)
	{
		cli();
		p=encoderMotorPos;
		sei();
		return p;
	}
	else
	{
		cli();
		p=(pasul+1-encoderMotorPos)*(-flag_directie);
		sei();
		return p;
	}
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
		static const uint8_t Kp=70;
		static const uint8_t Ki=10;
		static const uint8_t Kd=40;
		float Viteza_masurata;
		static float error;
		static float derivative;

		
		cli();
		lastMilli2 = millis();
		sei();
		
		Viteza_masurata=2158/ff[2];//filtrare viteza masurata
		error=viteza_referinta-Viteza_masurata/100;//setpoint - measured_value
		integral = integral + error * dt;
		derivative = (64.7/(ff[0]+ff[1]+ff[2]) - previous_error) / dt;
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
		if (flag_directie == 1)
		{
			mot.forward(output);
		}
		if (flag_directie == -1)
		{
			mot.reverse(output);
		}
		

//comandai2c(10, int(encoder_motor.directie()));//begin transmission, send (adress, int), end transmission
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
				led_red.off();
				flag_mod_setare_pozitie = true;
			}
			else{
				flag_mod_setare_pozitie = false;

			}
		}
		
		//ajustare microstep
		while (flag_mod_setare_pozitie == true)// daca flag_mod_setare_pozitie este adevarat, atunci se intra in mod de reglare pozitie
		{
			led_red.off();
			while (flag_directie == 1) {
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
					encoderMotorPos=1;
					led_red.off();
					delay(100);
				}
				if (buton_black.status() == 1) {
					cli();
					encoderMotorPos=1;
					sei();
					led_green.on();
					delay(500);
					flag_mod_setare_pozitie = false;
					flag_directie=0;
				}
			}
			//cod de la butoane
			if (flag_directie == -1) {
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
		cli();
		encoderMotorPos=1;
		sei();
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

int sleep_enablef(){// going to sleep is voltage BOD detection
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
		flag_directie=0;
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
		if (flag_directie == 1) {
			v |= (1 << j);
			flag_directie = 0;
			j--;
			i = 0;
		}
		if (flag_directie == -1) {
			v &= ~(1 << j);
			flag_directie = 0;
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
 
 
 ///Timer2 overflow interrupt
 ISR (TIMER2_OVF_vect){
 	TCNT2 = 205; //set timer new=205 C8
	ff[2] = 255;
 }
