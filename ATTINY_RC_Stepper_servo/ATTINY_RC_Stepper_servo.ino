/*
 * MODIFIED from  * ATTINY85_RC_Receiver.c
 *
 * Created: 01.01.2019 20:06:29
 * Author : Andreas
 * Description: Read and interpret signals from RC Receivers
 */ 

 /*
  /*
 * MODIFIED from  * ATTINY85_RC_Receiver.c
 *
 * Created: 01.01.2019 20:06:29
 * Author : Andreas
 * Description: Read and interpret signals from RC Receivers
 */ 

 /*
  * Dagnall 2021 modified to use alternate Timer
  * Board set to 8Mhz no usb
  * 
  * 
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

#define HALF_STEP true


#define StepSpeed 300 //uS per step //  slower for non half step? 

#define GAIN 15 //
#define RC_RECEIVER_PORT PB0
#define DEAD_ZONE GAIN*1.5




// drive to motor drive ic
#define OUTA PB1
#define OUT_An PB3
#define OUTB PB2
#define OUT_Bn PB4

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
    OSCCAL = 135;//142;  //Dag needs adjustng to ensure that servo pulses are seen ok. too high and one end will "fold back" in response                          // this gave 7,96MHz (peaks of 8MHz) for value ranges see datasheet page 52
}

void Init_PORT(void) {
    // this function initializes the pins which are used as output / inputs

    DDRB |= (1 << OUTA);                 // set led port as output
    DDRB |= (1 << OUT_An);  
    DDRB |= (1 << OUTB);  
    DDRB |= (1 << OUT_Bn);  
  
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
    // modified to use TCNT0 to allow delay to work.. 
    if ( RISING_EDGE ){       // check if rising edge pin change interrupt (beginning of servo pulse)
   //   TCNT1 = 0;                           // reset counter (TCNT1 page 91 TCNT0 page 78
   //   TCCR1 = (1 << CS12) | (1 << CS11) | (1 << CS10);   // start timer1 with prescaler CK/64 --> 250 steps per 2ms (TCCR1 page 89
        TCNT0 = 0; 
        TCCR0B = (1 <<CS01) | (1<< CS00) ; // start timero ck/64 page 80
        return;
       }
    // only reached when falling edge detected (end of servo pulse)
    //count = TCNT1;                           // take timer value to local variable
    //TCCR1 = 0;                               // stop timer
      count = TCNT0;                           // take timer value to local variable ~ 125-254 if osc correct
      TCCR0B = 0;                               // stop timer
      GIMSK &= ~(1 << PCIE);                   // Pin Change Interrupt Disable (datasheet page 51)
      pulse_ready=1;
}



void Stepper_Drive(int  in) {    // simple way to select 4 pin or 5 pin steppers etc 
    delayMicroseconds(StepSpeed);
//  Stepper4_LOW_Drive(in);
//  Stepper4_Drive(in);
   // Stepper4_half_step_Drive(in);
  if (HALF_STEP) {Stepper4_half_step_Drive(in);} else {Stepper4_Drive(in); } 
 
    
  }

void Stepper4_Drive ( int  in){
//   Serial.print(in);
  /* stepper 4 phase changes
   *  expects 0 - 3 input
 *  A+ A- B+ B- 
 *  1  0  0  0
 *  0  0  1  0 
 *  0  1  0  0
 *  0  0  0  1
 */
   uint8_t internal;
  internal= in % 4;    //(modulo keeps internal 0-3)
   switch (internal){
     case 0: digitalWrite(OUTA, HIGH);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, LOW);
            break;
    case 1:   digitalWrite(OUTA, LOW);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, HIGH);digitalWrite(OUT_Bn, LOW);
            break;
    case 2:   digitalWrite(OUTA, LOW);digitalWrite(OUT_An, HIGH);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, LOW);
            break;
    case 3:   digitalWrite(OUTA, LOW);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, HIGH);
            break;
        }
  }




void Stepper5_Drive(int  in) {

switch (in % 8)  {  //(modulo keeps internal 0-7)) 
case 0:
digitalWrite(OUTA, LOW);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, HIGH);
break;
case 1:
digitalWrite(OUTA, LOW);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, HIGH);digitalWrite(OUT_Bn, HIGH);
break;
case 2:
digitalWrite(OUTA, LOW);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, HIGH);digitalWrite(OUT_Bn, LOW);
break;
case 3:
digitalWrite(OUTA, LOW);digitalWrite(OUT_An, HIGH);digitalWrite(OUTB, HIGH);digitalWrite(OUT_Bn, LOW);
break;
case 4:
digitalWrite(OUTA, LOW);digitalWrite(OUT_An, HIGH);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, LOW);
break;
case 5:
digitalWrite(OUTA, HIGH);digitalWrite(OUT_An, HIGH);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, LOW);
break;
case 6:
digitalWrite(OUTA, HIGH);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, LOW);
break;
case 7:
digitalWrite(OUTA, HIGH);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, HIGH);
break;
default:
digitalWrite(OUTA, LOW);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, LOW);
break;
}

}

void Stepper4_half_step_Drive(int  in) {


switch (in % 8)  {  //(modulo keeps internal 0-7)) 
case 0:
digitalWrite(OUTA, HIGH);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, LOW);
break;
case 1:
digitalWrite(OUTA, HIGH);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, HIGH);digitalWrite(OUT_Bn, LOW);
break;
case 2:
digitalWrite(OUTA, LOW);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, HIGH);digitalWrite(OUT_Bn, LOW);
break;
case 3:
digitalWrite(OUTA, LOW);digitalWrite(OUT_An, HIGH);digitalWrite(OUTB, HIGH);digitalWrite(OUT_Bn, LOW);
break;
case 4:
digitalWrite(OUTA, LOW);digitalWrite(OUT_An, HIGH);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, LOW);
break;
case 5:
digitalWrite(OUTA, LOW);digitalWrite(OUT_An, HIGH);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, HIGH);
break;
case 6:
digitalWrite(OUTA, LOW);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, HIGH);
break;
case 7:
digitalWrite(OUTA, HIGH);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, HIGH);
break;
default:
digitalWrite(OUTA, LOW);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, LOW);
break;
}

}




void Achieved(){  
  PositionAchieved=true;digitalWrite(OUTA, LOW);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, LOW);
  }

void Move_To (int pos){  
  int diff,aim,deadzone; bool dir;
  deadzone=DEAD_ZONE;  // so we can use halfstep bool
  aim=(pos*GAIN); // use gain here! 
  if (HALF_STEP){aim=aim*2;deadzone=deadzone*2;}
  
  diff = aim - Stepper_Position;  
  if (abs(diff) >= (deadzone)){
    oldpos=Stepper_Position;
    dir=(abs(diff)== diff);
                 // should probably be a while.. loop, but I had trouble with that and this works.. 
  if (dir) {for (int x=oldpos; x<=aim;x++){Stepper_Position=x;Stepper_Drive(Stepper_Position);}}
       else{for (int x=oldpos; x>=aim;x--){Stepper_Position=x;Stepper_Drive(Stepper_Position); }}
      Achieved();           
     }
  }

 
     
  

void IOTEST(){ 
  if ( pulse_ready) {
         pulse_ready = 0;
         if ((count>=125)&&(count<=255)){
                   demand= int(count-125); //range is approx 125 to 255 gives 0-130
                   PositionAchieved=false; }
         GIFR = (1 << PCIF);              // clear Pin Change Interrupt Flag  (datasheet page 52)
         GIMSK |= (1 << PCIE); }           // Pin Change Interrupt Enable (datasheet page 51)
        }
  

void setup(){
    Calibrate_OSCILLATOR();
    Init_PORT();
   // send out nominal 50kHz for calibration
   //for (long x=0; x<=500000;x++){delayMicroseconds(1);digitalWrite(OUT_Bn, HIGH);delayMicroseconds(1);digitalWrite(OUT_Bn,LOW);}
   //
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
