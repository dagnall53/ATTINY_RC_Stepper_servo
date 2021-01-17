/*
 * MODIFIED from  * ATTINY85_RC_Receiver.c
 *
 * Created: 01.01.2019 20:06:29
 * Author : Andreas
 * Description: Read and interpret signals from RC Receivers
 */ 

 /*
  * Dagnall 2021 modified to use alternate Timer
  */

// the clock speed on ATTINY85 is approx. 8MHz
// if fuse CKDIV8 is set (factory default), a prescaler of 8 is used which results in a 1MHz clock
// for this code CKDIV8 needs to be unset as the code relies on 8MHz CPU speed
// the actual frequency can be measured at PB4 if the CKOUT fuse is set


//             ATTINY85 PIN Configuration (for more details see datasheet page 2)
//                  -------
//  RESET / PB5 ---| o     |--- VCC
//   ADC3 / PB3 ---|       |--- PB2 / SCK
//          PB4 ---|       |--- PB1 / MISO
//          GND ---|       |--- PB0 / MOSI
//                  -------

// includes
#include <avr/io.h>
#include <avr/interrupt.h>
//#include <util/atomic.h>


// *****************************
// ***** global variables ******
// *****************************

volatile uint8_t pulse_ready = 1;
volatile uint8_t count;
//volatile uint8_t overflows = 0;
int  Stepper_Position,oldpos;
uint8_t  demand;
//uint8_t local_overflows;
//uint8_t local_count;
int Timer1;
bool PositionAchieved;

#define StepSpeed  1000  //uS per step  min is about 750
#define DEAD_ZONE 22
#define GAIN 15 //20 sets  200 deg for 100 count
#define RC_RECEIVER_PORT PB0

#define OUT0 PB1
#define OUT1 PB2
#define OUT2 PB3
#define OUT3 PB4

#define OUT0_ON PORTB |= (1 << OUT0)
#define OUT1_ON PORTB |= (1 << OUT1)
#define OUT2_ON PORTB |= (1 << OUT2)
#define OUT3_ON PORTB |= (1 << OUT3)
#define OUT0_OFF PORTB &= ~(1 << OUT0)
#define OUT1_OFF PORTB &= ~(1 << OUT1)
#define OUT2_OFF PORTB &= ~(1 << OUT2)
#define OUT3_OFF PORTB &= ~(1 << OUT3)








#define RISING_EDGE PINB & (1 << RC_RECEIVER_PORT)

void Calibrate_OSCILLATOR(void){

    // function for calibrating the oscillator frequency
    
    // this program and corresponding timer setup requires that overflow to happens
    // therefore it is required to stay below 8MHz nominal frequency
    // specifically if the frequency is higher than 8MHz unwanted overflows would occur and destroy the software intend
    
    // Factory calibration for this very ATTINY85 was 148 (0x94 read from chip via ATMEL STUDIO "Device Programming")
    // this gave a measured frequency of 8,196MHz
    // frequency was measured with oscilloscope via PB4 (CKOUT fuse needs to be set for this)
    
    // OSCCAL needs to be calibrated per chip
    OSCCAL = 142;                            // this gave 7,96MHz (peaks of 8MHz) for value ranges see datasheet page 52
}

void Init_PORT(void) {
    // this function initializes the pins which are used as output / inputs

    DDRB |= (1 << OUT0);                 // set led port as output
    DDRB |= (1 << OUT1);  
    DDRB |= (1 << OUT2);  
    DDRB |= (1 << OUT3);  
  
    DDRB &= ~(1 << RC_RECEIVER_PORT);        // set receiver port as input
}


void OUT_Control(float position,uint8_t port){
    // this function switches the LED on depending on RC signal
    // position variable indicates 0%-100% stick position
    float loc;
    loc = position / 100 * 125;
     if ( count - 125 >= (int)loc ) {
        digitalWrite(port, LOW);
        //LED_ON;
    }
    else {
         digitalWrite(port, HIGH);
      
        //LED_OFF;
    }
}



void Init_INTERRUPTS(){
    // this function initializes the interrupts
     PCMSK |= (1 << PCINT0);                  // Pin Change Enable Mask for PB0 (pin 5) (datasheet page 52)
     sei();                                   // enable interrupts (MANDATORY)
}


ISR(PCINT0_vect){
    // interrupt service routine for pin change interrupt
    // if PINB is HIGH, a rising edge interrupt has happened
    // if PINB is LOW, a falling edge interrupt has happened

    if ( RISING_EDGE ){       // check if rising edge pin change interrupt (beginning of servo pulse)
   //  TCNT1 = 0;                           // reset counter (TCNT1 page 91 TCNT0 page 78
   //   TCCR1 = (1 << CS12) | (1 << CS11) | (1 << CS10);   // start timer1 with prescaler CK/64 --> 250 steps per 2ms (TCCR1 page 89
        TCNT0 = 0; 
        TCCR0B = (1 <<CS01) | (1<< CS00) ; // start timero ck/64 page 80
        return;
       }

    // only reached when falling edge detected (end of servo pulse)
    //count = TCNT1;                           // take timer value to local variable
    //TCCR1 = 0;                               // stop timer
      count = TCNT0;                           // take timer value to local variable 125-254 
      TCCR0B = 0;                               // stop timer
    
      GIMSK &= ~(1 << PCIE);                   // Pin Change Interrupt Disable (datasheet page 51)
      pulse_ready=1;
}



void Stepper_Drive(int  in) {
  //delay(StepSpeed);
  delayMicroseconds(StepSpeed);
  Stepper4_Drive(in);  // simple way to select 4 pin or 5 pin steppers
  }

void Stepper5_Drive(int  in) {

switch (in % 7)  {  //(modulo keeps internal 0-7)) 
case 0:
digitalWrite(OUT0, LOW);digitalWrite(OUT1, LOW);digitalWrite(OUT2, LOW);digitalWrite(OUT3, HIGH);
break;
case 1:
digitalWrite(OUT0, LOW);digitalWrite(OUT1, LOW);digitalWrite(OUT2, HIGH);digitalWrite(OUT3, HIGH);
break;
case 2:
digitalWrite(OUT0, LOW);digitalWrite(OUT1, LOW);digitalWrite(OUT2, HIGH);digitalWrite(OUT3, LOW);
break;
case 3:
digitalWrite(OUT0, LOW);digitalWrite(OUT1, HIGH);digitalWrite(OUT2, HIGH);digitalWrite(OUT3, LOW);
break;
case 4:
digitalWrite(OUT0, LOW);digitalWrite(OUT1, HIGH);digitalWrite(OUT2, LOW);digitalWrite(OUT3, LOW);
break;
case 5:
digitalWrite(OUT0, HIGH);digitalWrite(OUT1, HIGH);digitalWrite(OUT2, LOW);digitalWrite(OUT3, LOW);
break;
case 6:
digitalWrite(OUT0, HIGH);digitalWrite(OUT1, LOW);digitalWrite(OUT2, LOW);digitalWrite(OUT3, LOW);
break;
case 7:
digitalWrite(OUT0, HIGH);digitalWrite(OUT1, LOW);digitalWrite(OUT2, LOW);digitalWrite(OUT3, HIGH);
break;
default:
digitalWrite(OUT0, LOW);digitalWrite(OUT1, LOW);digitalWrite(OUT2, LOW);digitalWrite(OUT3, LOW);
break;
}

}
void Stepper4_Drive ( int  in){
//   Serial.print(in);
  /* stepper 4 phase changes
   *  expects 0 - 3 input
 *  A+ B+ A- B-
 *  1  0  0  1
 *  1  1  0  0
 *  0  1  1  0
 *  0  0  1  1
 */
   uint8_t internal;
  internal=in % 3;    //(modulo keeps internal 0-3)
   switch (internal){
     case 0:{digitalWrite(OUT0, HIGH);digitalWrite(OUT1, LOW);digitalWrite(OUT2, LOW);digitalWrite(OUT3, HIGH);
         }
    break;
    case 1:{ digitalWrite(OUT0, HIGH);digitalWrite(OUT1, HIGH);digitalWrite(OUT2, LOW);digitalWrite(OUT3, LOW);
         }
     break;
  case 2:{   digitalWrite(OUT0, LOW);digitalWrite(OUT1, HIGH);digitalWrite(OUT2, HIGH);digitalWrite(OUT3, LOW);
         }
     break;
    case 3:{ digitalWrite(OUT0, LOW);digitalWrite(OUT1, LOW);digitalWrite(OUT2, HIGH);digitalWrite(OUT3, HIGH);
         }
    break;
        }
  }

void Achieved(){  
  PositionAchieved=true;digitalWrite(OUT0, LOW);digitalWrite(OUT1, LOW);digitalWrite(OUT2, LOW);digitalWrite(OUT3, LOW);
}



void Move_To (int pos){  
  int diff,aim;
  aim=(pos*GAIN); // gain here! before counting set for geared test motor 20 sets  200 deg for 100 count
  diff = aim - Stepper_Position;  
  bool dir;
  if (abs(diff) >= (DEAD_ZONE)){
    oldpos=Stepper_Position;
    dir=(abs(diff)== diff);
                 // should probably be while loop, but I had trouble with that and this works.. 
  if (dir) {for (int x=oldpos; x<=aim;x++){Stepper_Position=x;Stepper_Drive(Stepper_Position);}}
       else{  for (int x=oldpos; x>=aim;x--){Stepper_Position=x;Stepper_Drive(Stepper_Position); }}
      Achieved();           
     }
  }

 
     
  

void IOTEST(){ 
  if ( pulse_ready) {
         pulse_ready = 0;
         if ((count>=125)&&(count<=255)){
                   demand= int(count-125); //range is approx 125 to 255
                   PositionAchieved=false; }
         GIFR = (1 << PCIF);              // clear Pin Change Interrupt Flag  (datasheet page 52)
         GIMSK |= (1 << PCIE); }           // Pin Change Interrupt Enable (datasheet page 51)
        }
  

void setup(){
    Calibrate_OSCILLATOR();
    Init_PORT();
    Init_INTERRUPTS();
    Stepper_Position=3200;
    demand=0;
    PositionAchieved=false;
    //Serial.begin(115200); 
    Move_To(0);  //to hit endstop..;
    ////oldpos=3200;for (int x=oldpos; x>=0;x--){Stepper_Position=x;Stepper_Drive(Stepper_Position);}  PositionAchieved=true;
  //calibration runs
 // delay(1000); Move_To(200);delay(500); Move_To(100);delay(500); Move_To(50);delay(500); Move_To(0);delay(1500); 
  //
  }

    
void loop(){
  
      
     if (!PositionAchieved) {  Move_To(demand);}
      IOTEST();
     
      
  }
