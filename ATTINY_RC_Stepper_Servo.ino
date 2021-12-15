  /*
  * Dagnall 2021 modified to use alternate Timer and drive stepper motor (I think this allows me to use  delay())..
  * Added code from http://becomingmaker.com/tuning-attiny-oscillator/ for interrupt based osc test frequency
  * MAINLY MODIFIED from  * ATTINY85_RC_Receiver.c
    * https://github.com/chiefenne/ATTINY85-RC-Receiver-Decoder
    * Created: 01.01.2019 20:06:29
    * Author : Andreas
    * Description: Read and interpret signals from RC Receivers
    * Board:     ATtiny25/45/85
    * Processor: Attiny85
    * Clock    : Internal 8MHZ
    * /////////////////////////////////
    * /// BURN BOOTLOADER FIRST!!! to set frequency ?////
    * /////////////////////////////////
   */ 
 
 
 // see Info.h for other info on project etc.. 
 /*
  * program with "USBTinyISP
  */



//#define  servo // define use as servo , else uses input pin as simple boolean left/right

// includes
#include <avr/io.h>
#include <avr/interrupt.h>

// *****************************
// ***** global variables ******
// *****************************

volatile uint8_t pulse_ready = 1;
volatile uint8_t count;

int  Stepper_Position,oldpos;
int  demand;
int Timer1;
bool PositionAchieved,Analog_mode;
bool LastInput;
int Counter;

//With an unknown board / motor the following cal routines can be set initially to help set the ATTiny clock and to discover the range of movement for the servo mechanism in steps 

// #define Send_10K_CAL
// #define Count_range_steps  // moves the drive repeatedly in 40 step increments so you can count the motor range.

//---------are we going to simulate an Analog servo, or a "End to end" device that switches at about PWM ==90 degrees? 

//#define ANALOG  //de comment for analog else unit drives end to end and uses approximatley mid point pwm as threshold (with hystresis)  
//#define _V1Board // correct errors on V1 board layou only!

#define REVERSE
#define BackOff // to reverese about 30 degrees for the American style knuckler 
float backangle = 0.1;  // portion of full scale
#ifdef servo
#define FullRange  1600 // ~1500 for exact full range movement on the linear servo and also defines travel for non analog  NOTE: larger value keeps motor driving until it hits endstops
#define GAIN 15
                      // GAIN is set to give full range for "0-100" (nominal 0-1ms)range (== 1.0 to 2.ms nominal response from the RC pulse as measured by the interrupt timer.
                      // so if the Range measurement is 36 counts (of 40 steps), the full range steps are 1440, say 1500?
                      // so gain should be 1500/100 = (integer!) 15 
#define StepSpeed 400   //uS per step 300 is about max speed at half step //  perhaps slower needed for full step? 

#define HALF_STEP true  //Full stepping (Halfstep false) at 350us gives about 8 oz thrust, BUT this is close to the dynamic max holding thrust;
                         // HAlf stepping gives slightly higher thrust and higher current drain..
                         // About 10 Oz seen, STATIC holding thrust is about 12-16 oz, 
                         // but once motor gets to max thrust, any "slip" can slip it back to about 8/9 oz before it holds again. 
 



#else
//now settings for switch mode 
#define FullRange  1000  // 1000 is 360 degrees for the Ultra-Tiny Micro Mini 6mm Planetary Gearbox 2-phase 4-wire Gear Stepper Motor 5V
#define GAIN 1
#define StepSpeed 400   //uS per step 300 is about max speed at full step for 6mm motor  //  600 at half step?
                        // takes about 200Ma max with the motor noted above 5V
#define HALF_STEP false
#endif

                        



void Calibrate_OSCILLATOR(void){// OSCCAL needs to be calibrated per chip
   //On chip oscillator Ideally needs adjustng (slowing down) to ensure that servo pulses are seen ok. too high and one end will "fold back" in response to >2ms pulses
   // because the interrupt register only gets to 2ms for 256 count, its a good idea to slow the whole clock down, to avoid "wrap around"
   // I have attempted to catch this in the count validation, but not entirely sucessfully. 
   // A better approach is to simply to slow the oscillator down deliberately. 
   // My Servo tester goes to 2.2ms , so we would need the clock to slow about 10% (min) to prevent wrap around. 
        
   //  for value ranges see datasheet page 31  note value ranges overlap at 127/128 ! so you may not get precisely the changes anticipated in this region.
   // Clock speed is approximately (50+(0.397*OSCAL))% of the nominal 8Mhz.


// EXAMPLE 63 is apparently about 75% speed (see datasheet) 
// this would give timing at 10.6us steps in the interrupt allowing measuring to about 2.7ms.  
  
    OSCCAL = 90;// sets to about 88% nominal clock..;  

    //ALSO note, I have a #define Send_10K_CAL to send out 10kHz on PB3
    // Then I can use a Freq counter on PB3 to check, measure and adjust OSCCAL if needed
    // BUT REMEMBER to re-comment  "Send_10K_CAL" before final programing 
   }


//PORTS define 


  #define RC_RECEIVER_PORT PB0
  // drive to motor drive ic
  //pinout for jlpcb board and V2  board
  #define OUTA PB1  // led is pb1
  #define OUT_An PB2    
  #ifdef REVERSE
     #define OUTB PB4
     #define OUT_Bn PB3
  #else
     #define OUTB PB3
     #define OUT_Bn PB4
  #endif


#define RISING_EDGE PINB & (1 << RC_RECEIVER_PORT)







void Init_PORT(void) {
  /*
 * Set 1 using DDRB |= (1 << PB3);
 * set 0 using PORTB &= ~(1 << PB3);
 * 
 * 
 * 
 */
  
  //?PORTB = (1<<PB0);

    // this function initializes the pins which are used as output / inputs
    DDRB |= (1 << OUTA);                 // set led port as output
    DDRB |= (1 << OUT_An);  
    DDRB |= (1 << OUTB);  
    DDRB |= (1 << OUT_Bn);  
  
    DDRB &= ~(1 << RC_RECEIVER_PORT);        // set receiver port as input
}


void OUT_Control(float position,uint8_t port){
    // this Andreas function switches port on depending on RC signal
    // position variable indicates 0%-100% stick position
    // ---------not used here but may be helpful in other applications ?relay drivers?
    float loc;
    loc = position / 100 * 125;
     if ( count - 125 >= (int)loc ) {
              digitalWrite(port, LOW);
              //LED_ON; etc
              }
        else {
              digitalWrite(port, HIGH);
              //LED_OFF; etc 
              }
}



void Init_INTERRUPTS(){
    // this function initializes the interrupts 
     PCMSK |= (1 << PCINT0);                  // Pin Change Enable Mask for PB0 (pin 5) (datasheet page 52)
     sei();                                   // enable interrupts (MANDATORY)
}


 #ifdef servo
 #ifdef Send_10K_CAL   // for the calibration oscillator set a timer interrupt. 
        ISR (TIMER0_COMPA_vect) {
         PORTB ^= 1 << PINB3;        // Invert pin PB3
        }
 #endif 


ISR(PCINT0_vect){   // I have retained the original code and comments from Andreas's code.
         // interrupt service routine for pin change interrupt to measure the RC servo pulse 
         // modified to use TCNT0 to allow delay to work.. 
    // if PINB is HIGH, a rising edge interrupt has happened
    // if PINB is LOW, a falling edge interrupt has happened
 
    if ( RISING_EDGE ){       // check if rising edge pin change interrupt (beginning of servo pulse)
   //   TCNT1 = 0;                           // reset counter (TCNT1 page 91 TCNT0 page 78
   //   TCCR1 = (1 << CS12) | (1 << CS11) | (1 << CS10);   // start timer1 with prescaler CK/64 --> 250 steps per 2ms (TCCR1 page 89
        TCNT0 = 0; 
        TCCR0B = (1 <<CS01) | (1<< CS00) ; // start timero ck/64 page 80 
        // (if "perfect oscillator", then 256 counts is 2.048ms .. ) so 1.5ms (mid) would be 187. 1ms should be 127 or so.
        // but if the pulse is > 2.048ms, it will "wrap" and possibly give errors if not caught  
        return;
       }
    // only reached when falling edge detected (end of servo pulse)
    //count = TCNT1;                           // take timer value to local variable
    //TCCR1 = 0;                               // stop timer
      count = TCNT0;                           // take timer value to local variable ~ aim for  125-254 for 1ms to 2ms 
      TCCR0B = 0;                              // stop timer
      GIMSK &= ~(1 << PCIE);                   // Pin Change Interrupt Disable (datasheet page 51)
      pulse_ready=1;
}
#endif


void Stepper_Drive(int  in) {    // simple way to select 4 pin or 5 pin steppers etc. 
    if (HALF_STEP) {delayMicroseconds(StepSpeed);Stepper4_half_step_Drive(in);}
              else {delayMicroseconds(StepSpeed);Stepper4_Drive(in);}
  }

void Stepper4_Drive ( int  in){
//   Serial.print(in);
  /* stepper 4 phase changes
   *  expects 0 - 3 input
 *  A+ A- B+ B- 
 *  1  0  0  0   <
 *  0  0  1  0   up
 *  0  1  0  0   >
 *  0  0  0  1   down
   0  0  1   <down
 
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



void Stepper4_HPDrive ( int  in){
//   EXPERIMENT IN higher power drive.. rotating 45,135,225,315 instead of 0,90,180,270
// Does not seem to give any higher torque in spite of higher current drain.
  /* stepper 4 phase changes
 *  L(A+ A-) B+ B- 
 *  1  0  1  0   =45
 *  0  1  1  0   =135
 *  0  1  0  1   =225 
 *  1  0  0  1   =315

 */
   uint8_t internal;
  internal= in % 4;    //(modulo keeps internal 0-3)
   switch (internal){
     case 0: digitalWrite(OUTA, HIGH);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, HIGH);digitalWrite(OUT_Bn, LOW);
            break;
    case 1:   digitalWrite(OUTA, LOW);digitalWrite(OUT_An, HIGH);digitalWrite(OUTB, HIGH);digitalWrite(OUT_Bn, LOW);
            break;
    case 2:   digitalWrite(OUTA, LOW);digitalWrite(OUT_An, HIGH);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, HIGH);
            break;
    case 3:   digitalWrite(OUTA, HIGH);digitalWrite(OUT_An, LOW);digitalWrite(OUTB, LOW);digitalWrite(OUT_Bn, HIGH);
            break;
        }
  }




void Stepper5_Drive(int  in) { // for 5 wire motors as in educational kits

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
  aim=(pos*GAIN);     // move this many steps per "unit" (circa 0.1ms/count) of Rc pulse width .. (range ~ 0-130, see iotest) 
  deadzone=GAIN*2;  // allow a deadzone for no response if only a small change is seen in the demand, to prevent "hunting" and allow drive to switch off
  
  // how far from last position?  
  
  if (HALF_STEP){aim=aim*2; deadzone= deadzone*2;} 
  diff = aim - Stepper_Position; 
  if (abs(diff) >= (deadzone)){
    oldpos=Stepper_Position;
    dir=(abs(diff)== diff);
                  // should probably be a while.. loop, but I had trouble with that and this works.. 
    if (dir) {for (int x=oldpos; x<=aim;x++){Stepper_Position=x;Stepper_Drive(Stepper_Position);}}
         else{for (int x=oldpos; x>=aim;x--){Stepper_Position=x;Stepper_Drive(Stepper_Position); }}
    Achieved();  // leaves current position in Stepper_Position         
    }
  }

void Move_To_ABS (int pos){  
  int diff,aim; bool dir;
  // No gains here! this code used only move absolute position mainly for range testing, no deadzone either
  diff = pos - Stepper_Position;  
    oldpos=Stepper_Position;
    dir=(abs(diff)== diff);
    if (HALF_STEP){aim=aim*2;}
                 // should probably be a while.. loop, but I had trouble with that and this works.. 
  if (dir) {for (int x=oldpos; x<=aim;x++){Stepper_Position=x;Stepper_Drive(Stepper_Position);}}
       else{for (int x=oldpos; x>=aim;x--){Stepper_Position=x;Stepper_Drive(Stepper_Position); }}
      Achieved();           
  }

void Count_Motor_Range(){ // Move motor absolute 40 steps to explore range. 
  for (int x=0;x<=40;x++){
    delay(1000); Move_To_ABS(x*40);}
    Move_To_ABS(0);
}
  

void IOTEST(){ // measure the RC sevo signal
  int Hystresis;
  if ( pulse_ready) {
         pulse_ready = 0;
         if ((count>=100)&&(count<=250)){  //  this bit could do with adjustment to accomodate the deliberate osccal changes? 124 to 256 for 1ms to 2ms
             if (Analog_mode){     
                   demand= int(count-100); //Nominal servo range is approx 1ms to 2ms  (My digital tester allows  0.8ms to 2.2ms)
                   PositionAchieved=false; }
             else {  // end to end switched mode...add hystresis to avoid hunting Range Stepper_Position= 0 or Fullrange;?
                   Hystresis = 0; demand=0;
                   if (Stepper_Position>=50){Hystresis = 20;}
                   if (count>=(170-Hystresis)){demand=(2*FullRange)/GAIN;}  // drive to twice range 
                   PositionAchieved=false;
                          }
          } 
          GIFR = (1 << PCIF);   // clear Pin Change Interrupt Flag  (datasheet page 52)
          GIMSK |= (1 << PCIE); // Pin Change Interrupt Enable (datasheet page 51)
      }           
  }

void Int_driven_10k(){
    Calibrate_OSCILLATOR();
    pinMode(3,OUTPUT);          // Set PB3 to output
    TCNT0 = 0;                  // Count up from 0
    TCCR0A = 2 << WGM00;        // CTC mode
    if (CLKPR == 3)             // If clock set to 1MHz
        TCCR0B = (1<<CS00);     // Set prescaler to /1 (1uS at 1Mhz)
    else                        // Otherwise clock set to 8MHz
        TCCR0B = (2<<CS00);     // Set prescaler to /8 (1uS at 8Mhz)
    GTCCR |= 1 << PSR0;         // Reset prescaler
    OCR0A = 49;                 // 49 + 1 = 50 microseconds (10KHz)
    TIFR = 1 << OCF0A;          // Clear output compare interrupt flag
    TIMSK |= 1 << OCIE0A;       // Enable output compare interrupt
}

void Step_set_Zero(int Flashes){
// for identify at startup
#ifdef REVERSE

for (int x=1; x<=Flashes; x++){
      delay(50);
      Stepper_Position=(2*FullRange)/Flashes; //to hit endstop in flashes steps, aiming at 2x Fullrange ..;
      demand=0;
      PositionAchieved=false;
      Move_To_ABS(0);  
      }
#else

for (int x=1; x<=Flashes; x++){
      delay(50);
      Stepper_Position=0; //to hit endstop in flashes steps, aiming at 2x Fullrange ..;
      demand=(2*FullRange)/Flashes;
      PositionAchieved=false;
      Move_To_ABS((2*FullRange)/Flashes);  
      }
#endif

      
}



void setup(){
  #ifdef Send_10K_CAL 
         Int_driven_10k();
  #else
  
   Analog_mode= false;
    
    #ifdef ANALOG
      Analog_mode= true;
    #endif
  if ((analogRead(0)) <= 900 ) {  Analog_mode= true;}  // 10 bit test using RST /..  if nothing connected, reset pin is near Vcc else is lower..    
   Calibrate_OSCILLATOR();
   Init_PORT();
   
#ifdef servo 
   Init_INTERRUPTS();
   
     if (Analog_mode){
        Step_set_Zero(4); // 4 flashes tells you its analog mode
        } 
        else{Step_set_Zero(1);
        }
  #endif      
   //calibration runs ?
   #ifdef Count_range_steps
         Count_Motor_Range();
   #endif
    
  #endif
  LastInput= digitalRead(RC_RECEIVER_PORT);
  
  if   (digitalRead(RC_RECEIVER_PORT)){ if (HALF_STEP){Stepper_Position=(FullRange*2/GAIN); }else {Stepper_Position=(FullRange/GAIN);}}
               else {Stepper_Position=0;}
  Counter=0;
  Achieved();
 }

    
void loop(){
//Servo loop
#ifdef servo 
  if (!PositionAchieved) {  Move_To(demand);    }
          IOTEST();
#else
// end servo loop 

// if   (digitalRead (RC_RECEIVER_PORT) ==true){Move_To(100);
// delay (10);
// Move_To(0);
// delay(10);
// }
// else{Move_To(5);
// delay (2000);Move_To(0);
// delay(2000);
// 
//  }
 bool State=digitalRead(RC_RECEIVER_PORT);
 
  if (State == LastInput){Counter=0;}
  Counter++;
  if (Counter >= 100){  // state changeed!   count is debounce !
          LastInput=State;
          
          if   (State==true){ 
              Move_To(FullRange/GAIN); 
                               #ifdef BackOff  
                               Move_To((1-backangle)*(FullRange/GAIN)); // about 30 degrees backing off
                               #endif
            }else {Move_To(0);
                               #ifdef BackOff  
                               Move_To(backangle*(FullRange/GAIN)); // about 30 degrees backing off
                               #endif
                               }
          
             
               
  }
  delay(1);
 

#endif
}
      
