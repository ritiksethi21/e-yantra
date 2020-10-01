#define F_CPU 16000000
#define __DELAY_BACKWARD_COMPATIBLE__


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include "mcp23017.h"
#include "VL53L0X.h"
#include "uart.h"
#include "i2c.h"
#include "millis.h"

#define xshut_1 PC4
#define xshut_2	PG1
#define xshut_3 PL7
#define PIN_LED_RED PH3
#define PIN_LED_GREEN PH5
#define PIN_LED_BLUE PH4
#define max_speed 120

char path[100];
int final_direct[100];
int final_step[100];
int curr_direct=3,first_cell=0,last_cell=0,lfa_flag=0;
uint16_t Sensor[3]; 
volatile unsigned int count = 0,hos_count=0;


uint8_t addr[3]={22,24,26};    // array with new addresses for 3 TOF Sensors

void init_Xshut_Low(void)
{
	DDRC |= (1<<xshut_1);
	PORTC &= ~(1<<xshut_1);
	DDRG |= (1<<xshut_2);
	PORTG &= ~(1<<xshut_2);
	DDRL |= (1<<xshut_3);
	PORTL &= ~(1<<xshut_3);
}

// initialize I2C, MILLIS and UART
void init_i2c_uart(void)
{
	i2c_init();
	initMillis();
	
	sei();
}

// initialise all 3 TOF sensors and change their default address
void init_Sensor_Vl53l0x(){
	init_Xshut_Low();

	DDRC &= ~(1<<xshut_1);
	// initialize First VL53L0X sensor
	initVL53L0X(1);
	// Change the address to "addr[0] = 22"
	setAddress(addr[0]);

	DDRG &= ~(1<<xshut_2);
	// initialize Second VL53L0X sensor
	initVL53L0X(1);
	// Change the address to "addr[1] = 24"
	setAddress(addr[1]);

	DDRL &= ~(1<<xshut_3);
	// initialize Third VL53L0X sensor
	initVL53L0X(1);
	// Change the address to "addr[0] = 26"
	setAddress(addr[2]);
}

void led_init(void){
DDRH    |= (1 << PIN_LED_RED) | (1 << PIN_LED_GREEN) | (1 << PIN_LED_BLUE);    
PORTH   |= (1 << PIN_LED_RED) | (1 << PIN_LED_GREEN) | (1 << PIN_LED_BLUE);    
}

void led_redOn(void){
PORTH &= ~(1 << PIN_LED_RED);
}

void led_redOff(void){
PORTH |= (1 << PIN_LED_RED);
}

void led_greenOn(void){
PORTH &= ~(1 << PIN_LED_GREEN);
}

void led_greenOff(void){
PORTH |= (1 << PIN_LED_GREEN);
}

void led_blueOn(void){
PORTH &= ~(1 << PIN_LED_BLUE);
}

void led_blueOff(void){
PORTH |= (1 << PIN_LED_BLUE);
}

void init_lfa(void){
	uint8_t address = 0b000;	// A2 A1 A0 = 000

	// initialize MP23017
	mcp23017_init();

	// set all pins of MCP23017 PORTA as INPUT
	mcp23017_setmodeA(address, MCP23017_MODEINPUTALL);

	// set pin 0 of MCP23017 PORTB (GPB.0) as OUTPUT
	mcp23017_setmodepinB(address, 0, MCP23017_MODEOUTPUT);

	// set pin 1 of MCP23017 PORTB (GPB.1) as OUTPUT
	mcp23017_setmodepinB(address, 1, MCP23017_MODEOUTPUT);

	// make pin 0 of MCP23017 PORTB (GPB.0) as LOW
	mcp23017_writepinB(address, 0, MCP23017_PINSTATEOFF);

	// make pin 1 of MCP23017 PORTB (GPB.1) as LOW
	mcp23017_writepinB(address, 1, MCP23017_PINSTATEOFF);

}

void init_timer2(void){
    cli();  // Turn off global interrupts

    //Setup Timer2 to fire every 1ms
    TCCR2B = 0x00;                              // Cut off Clock Source to disbale Timer2 while we set it up
    TCNT2  = 130;                               // Reset Timer Count to 130 out of 255
    TIFR2  &= ~(1 << TOV2);                     // Timer2 INT Flag Reg: Clear Timer Overflow Flag
    TIMSK2 |= (1 << TOIE2);                     // Timer2 INT Reg: Timer2 Overflow Interrupt Enable
    TCCR2A = 0x00;                              // Timer2 Control Reg A: Wave Gen Mode normal
    TCCR2B |= (1 << CS22) | (1 << CS20);        // Timer2 Control Reg B: Timer Prescaler set to 128 and Start Timer2

    sei();  // Turn on global interrupts
}


//Timer2 Overflow Interrupt Vector
ISR(TIMER2_OVF_vect) {
  count++;  // increment after 1 ms               
  
  // increment seconds variable after 1000 ms
  if(count > 180){
    OCR5AL = 255;
    OCR5BL = 255;
    OCR1AL = 255;
    OCR1BL = 255;
    count = 0; 
	TCCR2B=0x00;  
  }
  TCNT2 = 130;              // Reset Timer to 130 out of 255
  TIFR2  &= ~(1 << TOV2);
};

void timer5_init(){
cli();  
TCCR5A |= (1 << COM5B1);
TCCR5A |= (1 << COM5A1);
TCCR5A |= (1 << WGM50);
TCCR5B |= (1 << WGM52);
TCCR5B |= (1 << CS51) | (1 << CS50);
sei();
}
//using timer2 to control a motor (in pwm mode)
void timer1_init(){
cli();  
TCCR1A |= (1 << COM1B1);
TCCR1A |= (1 << COM1A1);
TCCR1A |= (1 << WGM10);
TCCR1B |= (1 << WGM12);
TCCR1B |= (1 << CS11) | (1 << CS10);
sei();
}
// initialize respective timemrs and motor pins from operating motor
void init_motors(void){
timer1_init();
timer5_init();
DDRL |=(1<<PL3) | (1<<PL4);
PORTL|=(1<<PL4) | (1<<PL3);
DDRB |=(1<<PB5) | (1<<PB6);
PORTB|=(1<<PB5) | (1<<PB6);

OCR5AL = 255;
OCR5BL = 255;
OCR1AL = 255;
OCR1BL = 255;
}

int lfa()
{
    uint8_t x;
    x=mcp23017_readpinsA(0);
    if(x==0)
    lfa_flag=0;
    if(lfa_flag==1)
    return 0;
    else
    return x;
   }
char uart0_readByte(void){

	uint16_t rx;
	uint8_t rx_status, rx_data;

	rx = uart0_getc();
	rx_status = (uint8_t)(rx >> 8);
	rx = rx << 8;
	rx_data = (uint8_t)(rx >> 8);

	if(rx_status == 0 && rx_data != 0){
		return rx_data;
	} else {
		return -1;
	}

}

ISR(INT7_vect)
{
	uart0_puts("@started@");
}

void init_switch_interrupt(void){
	
	DDRE |=(1<<PE7);
	PORTE |=(1<<PE7);
	cli();

	EIMSK |= (1 << INT7);	
	
	EICRB |= (1 << ISC71);	
	EICRB &= ~(1 << ISC70);

	sei();	
}

void stop()
{
OCR5BL = 255;
OCR5AL = 255;
OCR1AL = 255;
OCR1BL = 255;
}

void forward()
{
OCR5BL = 160;
OCR5AL = 255;
OCR1AL = 255;
OCR1BL = 160;
}

void left()
{
OCR5AL = 255;
OCR5BL = 160;
OCR1BL = 255;
OCR1AL = 160;
_delay_ms(12);
forward();
}
void right()
{
OCR5BL = 255;
OCR5AL = 160;
OCR1AL = 255;
OCR1BL = 160;
_delay_ms(12);
forward();
}
void right_90d(void){
    OCR5AL = 70;
    OCR5BL = 255;
    OCR1BL = 70;
    OCR1AL = 255;
    count=0;
    init_timer2();
    _delay_ms(500);
}
void left_90d(void){
    OCR5AL = 255;
    OCR5BL = 70;
    OCR1BL = 255;
    OCR1AL = 70;
    count=0;
    init_timer2();
    _delay_ms(500);
}
void turn(int var)
 {	
	stop();
	if(var==0)
		right_90d();
	else left_90d();
	stop();
	TCCR2B=0x00;
	_delay_ms(1000);
}

void stright(int lfa_count,int var)
{
	int p=0;
if(lfa_count==1)
p=1;
uint16_t Sensor[2],ref_for,curr_for;
if(var==1 && last_cell!=1)
lfa_count--;
while(lfa_count)
{
        forward();
        if(lfa())
        {
        lfa_flag=1;
        lfa_count--;
        forward();
        }
		for(int i=0;i<2;i++)
		{
		Select_ToF_Addr(addr[i]);                
		Sensor[i]=readRangeSingleMillimeters(0);
		}
		if(Sensor[1]<120)
        {
            if(Sensor[1]<65)
            right();
            else if(Sensor[1]>90)
            left();
        }
        else if(Sensor[0]<120)
        {
            if(Sensor[0]<65)
            left();
            else if(Sensor[0]>90)
            right();
        }
		 	}
            led_blueOn();
    stop();
    forward();
	if(var!=1)
	{
    Select_ToF_Addr(addr[2]);                
	ref_for=readRangeSingleMillimeters(0);
	if(p!=1)
	{
    if(ref_for<200)
    while(1)
    {
        Select_ToF_Addr(addr[2]);                
	    curr_for=readRangeSingleMillimeters(0);
        if(abs(curr_for-ref_for)>70)
        break;
    }
    else
    {
    forward();
	_delay_ms(250);
	}
	}
	else
	{
	forward();
	_delay_ms(500);	
	}
	
	led_blueOff();
	}
TCCR2B=0x00;
stop();
_delay_ms(1000);
}
void final_function(int n)
{
	int k,i,x=0;
	for(i=0;i<n;i++)
	{
		if(i==n-1)
		x=1;
		k=final_direct[i]-curr_direct;
		if(k==0)
		{
		stright(final_step[i],x);
		}
		else if(abs(k)==2)
		{
			led_greenOn();
			_delay_ms(4000);
			led_greenOff();
			stright(final_step[i],x);
		}
		else if(k==1 || k==-3)
		{
			turn(0);
			stright(final_step[i],x);
		}
		else 
		{
			turn(1);
			stright(final_step[i],x);
		}
		curr_direct=final_direct[i];
		}
		if(last_cell==0)
		{
		led_redOn();
		stop();
		_delay_ms(1000);
		led_redOff();
		uart0_puts("firezone reached");
		}
		memset(path,0,strlen(path)*sizeof(path[0]));
		memset(final_direct,0,n*sizeof(final_direct[0]));
		memset(final_step,0,n*sizeof(final_step[0]));
	}

void path_finder(void)
{
	int direct[100];
	int i,len=strlen(path),x,y,next_x,next_y,new_count=1,flag=0;
	int count=0,p=0,arr1[100][2];
	for(i=0;i<len;i++)
	{
		if(path[i]>=48 && path[i]<=58)
		{
			if(p!=2)
			arr1[count][p]=path[i]-'0';
			else
			{
			count++;
			p=0;
			arr1[count][p]=path[i]-'0';
			}
			p++;
		}
		}
	if(arr1[count][0]==9 && arr1[count][1]==9){
	last_cell=1;
	}
	if((arr1[0][0])!=0 || (arr1[0][1])!=0)
			{
				for(int i=0;i<=count;i++)
				{
					arr1[i][0]=arr1[i+1][0];
					arr1[i][1]=arr1[i+1][1];
					}
					count--;
				}
	x=arr1[0][0];
	y=arr1[0][1];
	for(i=1;i<=count;i++)
	{
		next_x=arr1[i][0];
		next_y=arr1[i][1];
		if(next_x==x)
		{
			if(next_y>y)
			direct[i-1]=3;
			else
			direct[i-1]=1;
			}
		else
		{
			if(next_x>x)
			direct[i-1]=0;
			else
			direct[i-1]=2;
		}
		x=next_x;
		y=next_y;
		}
		for(i=0;i<count;i++)
		{
			if(direct[i]==direct[i+1])
				new_count++;
			else
			{
				final_direct[flag]=direct[i];
				final_step[flag]=new_count;
				flag++;
				new_count=1;
				}
			
			}
		final_function(flag);	
}
void receive_path()
{
	volatile unsigned int count = 0;	
	char rx_byte;
		while(1)
		{
		rx_byte = uart0_readByte();
		if(rx_byte != -1){
			if(rx_byte=='#' && count!=0)
			break;
			else
			{
			path[count]=rx_byte;	
			count++;
			}
		}
	}
	path_finder();
	count=0;
}

void hos_right(int x)
{
    OCR1AL = 255;
    OCR5BL = 255;
    OCR1BL = 100;
    OCR5AL = 100;
    _delay_ms(x);
    OCR1BL = 255;
    OCR5AL = 255;
}
void hos_left(int x)
{
    OCR1AL = 100;
    OCR5BL = 100;
    OCR1BL = 255;
    OCR5AL = 255;
    _delay_ms(x);
    OCR1AL = 255;
    OCR5BL = 255;
}
void run(int speed , int x){
    OCR1BL = speed;
    OCR5AL = 255;
    OCR1AL = 255;
    OCR5BL = speed;
    _delay_ms(x);
    OCR1BL = 255;
    OCR5AL = 255;
    OCR1AL = 255;
    OCR5BL = 255;
}

void hospital()
{	TCCR2B=0x00;
	if(curr_direct==0)
	{
		led_blueOn();		//left_turn
		_delay_ms(4000);
		led_blueOff();
	}
	 while(1){
        OCR1BL = 150;
        OCR5AL = 255;
        OCR1AL = 255;
        OCR5BL = 150;
        Select_ToF_Addr(addr[1]);
        uint16_t x = readRangeSingleMillimeters(0);
        Select_ToF_Addr(addr[0]);
        uint16_t y = readRangeSingleMillimeters(0);
        
        if(x > 200) {
            stop();
            _delay_ms(1000);
        }
        if(x<60){
           hos_right(70-x);
            run(150,10);
        }
        else if(x>100){
            hos_left(x-90);  
            run(150,10);
        }
        if(y<150) {
			forward();
			_delay_ms(300);
            stop();
			right_90d();
            led_greenOn();
            _delay_ms(1000);
            led_greenOff();
			left_90d();
			if(hos_count==0)
			uart0_puts("HA reached!");
			else if(hos_count==1)
			uart0_puts("HB reached!");
			else 
			uart0_puts("HC reached,Task Accomplished!");
			forward();
			_delay_ms(300);
			hos_count++;
        }
		if(hos_count==3)
		{
			stop();
			break;
		}
    }
}

int main(void) {

	uart0_init(UART_BAUD_SELECT(115200, F_CPU));
	uart0_flush();
	init_motors();
	led_init();
	init_switch_interrupt();
	init_lfa();
	init_i2c_uart();
	init_Sensor_Vl53l0x();
	while(1){
		if(last_cell == 1) break;
		receive_path();
	}
	forward();
	_delay_ms(600);
	stop();
	left_90d();
	stop();
	forward();
	_delay_ms(600);
	hospital();
	return 0;	
}
