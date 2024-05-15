/*
* final project
*  Created on: 15/5/2024
*  Author: Ali Ahmed
*/
#include <avr/io.h>
#include <avr/interrupt.h>

/* global variables contain the time value which will be dispalyed on the 7 seg */
unsigned char Hours=0,Minutes=0,Seconds=0;
/* global variable contain the value of the speed of motor */
unsigned char duty_cycle=128;
/* global variables used in turn on and off */
unsigned char PIR_Pressed = 0,display_on = 0;


ISR(TIMER1_COMPA_vect){
    Seconds++;
    // check if Seconds equal to 60
    if(Seconds == 60){
      Seconds = 0;
      Minutes++;
    }
    // check if Minutes equal to 60
    if(Minutes == 60){
      Minutes = 0;
      Hours++;
    }
    // check if Hours equal to 24
    if(Hours == 24){
      Hours = 0;
    }
}
// increase the speed of motor
ISR(INT0_vect){
  duty_cycle = duty_cycle+25;
  if(duty_cycle == 2556){
    duty_cycle =0;
  }
}
// decrease the speed of motor
ISR(INT2_vect){
  duty_cycle = duty_cycle-25;
}
// turn on & off the treadmile 
ISR(INT1_vect){
  if(PIR_Pressed == 0){
    PIR_Pressed = !PIR_Pressed;
    display_on = !display_on;
    Timer1_CTC();
    TIMER0_PWM(duty_cycle);
    INT0_init();
    INT2_init();
  }
  else{
    PIR_Pressed = !PIR_Pressed;
    DDRB  &= (~(1<<0)) & (~(1<<1));
    TCCR1B &= (~(1<<CS10)) & (~(1<<CS12));
    TCCR0 &= (~(1<<CS01));
    Hours=0;
    Minutes=0;
    Seconds=0;
    display_on = !display_on;
  } 
}
void INT0_init(void){
  GICR  |= (1<<INT0);//enable INT0
  MCUCR |= (1<<ISC01);//falling edge of INT0
  SREG   |=(1<<7);//enable interrupt
}
void INT2_init(void){
  GICR   |= (1<<INT2);//enable INT2
  MCUCSR |= ((1<<ISC2));//rising edge of INT0
  SREG   |=(1<<7);//enable interrupt
}
void INT1_init(void){
  MCUCR |= (1 << ISC11); //falling edge of INT0
  GICR |= (1 << INT1);//enable INT1
  SREG |= (1 << 7);//enable interrupt
}
void Timer1_CTC(){
  TCCR1B |= (1<<WGM12) | (1<<CS10) | (1<<CS12);// prescalar of 1024 & CTC Mode
  TCCR1A |= (1<<FOC1A);//non PWM Mode
  TCNT1   = 0;
  OCR1A   = 1000;
  TIMSK  |= (1<<OCIE1A);//enable timer iterrupt
  SREG   |=(1<<7);//enable interrupt
}
void TIMER0_PWM(unsigned int Duty_Cycle){
  TCNT0 = 0;
  OCR0 = Duty_Cycle; 
  TCCR0 |= (1<<WGM00) | (1<<WGM01) | (1<<COM01) | (1<<CS01);//prescalar of 8 & Fast PWM &non-inverting mode)
  DDRB |= (1<<PB3); // make it as OUTPUT
}
int main(void){
  DDRD  &= (~(1<<PD3));// make it as INPUT
  DDRC  |= 0x0F;// make the first 4 pins  as OUTPUT
  PORTC &= 0xF0;// make the last 4 pins  as INPUT
  DDRA  |= 0x3F;// make the first 6 pins  as OUTPUT
  PORTA = 0x00;
  DDRB  &= (~(1<<2)) & (~(1<<4)) & (~(1<<5));// make these pins  as INPUT
  DDRB  |= (1<<0) | (1<<1);// make these pins  as OUTPUT
  PORTB |= (1<<0);// make it as OUTPUT
  INT1_init(); // call intialization of INT1
  while(1){
    if (display_on){//if display_on is equal to 1 the 7 Seg will turn on
      PORTA = (1<<0);
      PORTC = (PORTC & 0xF0) | ((Hours/10) & 0x0F);
      _delay_ms(2);
      PORTA = (1<<1);
      PORTC = (PORTC & 0xF0) | ((Hours%10) & 0x0F);
      _delay_ms(2);
      PORTA = (1<<2);
      PORTC = (PORTC & 0xF0) | ((Minutes/10) & 0x0F);
      _delay_ms(2);
      PORTA = (1<<3);
      PORTC = (PORTC & 0xF0) | ((Minutes%10) & 0x0F);
      _delay_ms(2);
      PORTA = (1<<4);
      PORTC = (PORTC & 0xF0) | ((Seconds/10) & 0x0F);
      _delay_ms(2);
      PORTA = (1<<5);
      PORTC = (PORTC & 0xF0) | ((Seconds%10) & 0x0F);
      _delay_ms(2);
    }
    else{//if display_on is equal to 0 the 7 Seg will turn off
      PORTC &= 0x00;
      PORTA = (PORTA)& 0x00;
    }
    if((PINB & (1<<4))){//if PB4 is equal to 1 the timer will resume
      resume_time();
    }
    if((PINB & (1<<5))){//if PB5 is equal to 1 the timer will stop
      paused_time();
    }
    OCR0 = duty_cycle;
  }
}
void paused_time(void){
  TCCR1B &= (~(1<<CS10)) & (~(1<<CS12));//make the timer stop
}
void resume_time(void){
  TCCR1B |= (1<<CS10) | (1<<CS12);//make the timer prescaler 1024 again
}