
/*
 * MODIFIED from  * ATTINY85_RC_Receiver.c
 * https://github.com/chiefenne/ATTINY85-RC-Receiver-Decoder
 * Created: 01.01.2019 20:06:29
 * Author : Andreas
 * Description: Read and interpret signals from RC Receivers
 */ 

 /*
  * Dagnall 2021 modified to use alternate Timer and drive stepper motor
  * Board set to internal 8Mhz no usb
  * 
  * Added code from http://becomingmaker.com/tuning-attiny-oscillator/ for interrupt based osc test frequency
  * 
  * I programmed using David A Mellis 's Board code,  
  * Board:     ATtiny25/45/85
  * Processor: Attiny85
  * Clock    : Internal 8MHZ
  * 
  * Programmer: Arduino as ISP 
  * via an arduino programmer (Programmed using "ArduinoISP" from EXAMPLES)
  *  (NOTE- Burn Bootstrap the first time on the ATTINY85!)
  * https://www.instructables.com/How-to-Program-an-Attiny85-From-an-Arduino-Uno/
  * 
  * set programmer to clock internal 8MHz
  * 
  * I believe this makes the comments about CKDIV* in the orinal code irrelevant.. 
  * "" if fuse CKDIV8 is set (factory default), a prescaler of 8 is used which results in a 1MHz clock
  *    for this code CKDIV8 needs to be unset as the code relies on 8MHz CPU speed
  *    the actual frequency can be measured at PB4 if the CKOUT fuse is set""
  * 
  * 
  * 
  */

  // the clock speed on ATTINY85 is approx. 8MHz
  // 
  // datasheet https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2586-AVR-8-bit-Microcontroller-ATtiny25-ATtiny45-ATtiny85_Datasheet.pdf

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

// *****************************
// ***** global variables ******
// *****************************

volatile uint8_t pulse_ready = 1;
volatile uint8_t count;

int  Stepper_Position,oldpos;
uint16_t  demand;
int Timer1;
bool PositionAchieved,Analog_mode;

//With an unknown board / motor the following cal routines need to be set initially to help set the ATTiny clock and to discover the range of movement for the servo mechanism in steps 

// #define Send_10K_CAL
// #define Count_range_steps  // moves the drive repeatedly in 40 step increments so you can count the motor range.

//#define ANALOG  //de comment for analog else end to end 

#define FullRange 3000  // ~3000 for full range movement option for non analog  larger keeps motor driving until it hits endstops
 
#define StepSpeed 600 //600uS per step is ok 300 is about max speed at half step //  perhaps slower needed for full step? 
#define HALF_STEP true

// GAIN is set to give full range for "0-125" response from the RC pulse as measured by the interrupt timer.
// so if the Range measurement is 36 counts (of 40 steps), the full range steps are 1440
// so gain sould be 1440/125 = (integer!) 12 
#define GAIN 12 

void Calibrate_OSCILLATOR(void){// OSCCAL needs to be calibrated per chip
    // un comment #define Send_10K_CAL to send out 10kHz on PB3
    // Then use Freq counter on PB3 to measure and adjust OSCCAL accordingly
    // re-comment  "Send_10K_CAL" before final programing 
// revised comments, because the interrupt register only gets to 2ms for 256, its a good idea to slow the whole clock down, to avoid "wrap around"
// My Servo tester goes to 2.2ms , so we would need the clock to slow about 10% (min) to prevent wrap around. 
// 63 is apparently about 75% speed (see datasheet) 
// this would give timing at 10.6us steps in the interrupt allowing measuring to about 2.7ms. 
#define OSCNOM 88    
    OSCCAL = 97;// sets to about 88% nominal clock..;  //needs adjustng (slowing down) to ensure that servo pulses are seen ok. too high and one end will "fold back" in response to >2ms pulses      
   //  for value ranges see datasheet page 31  note value ranges overlap at 127/128 ! so you may not get precisely the changes anticipated in this region.
   // Clock speed is approximately (50+(0.397*OSCAL))% of the nominal 8Mhz.
   }


//PORTS 
#define RC_RECEIVER_PORT PB0
// drive to motor drive ic
//pinout for jlpcb board 
#define OUTA PB1
#define OUT_An PB2
#define OUTB PB3
#define OUT_Bn PB4

#define RISING_EDGE PINB & (1 << RC_RECEIVER_PORT)

void Init_PORT(void) {
    // this function initializes the pins which are used as output / inputs
    DDRB |= (1 << OUTA);                 // set led port as output
    DDRB |= (1 << OUT_An);  
    DDRB |= (1 << OUTB);  
    DDRB |= (1 << OUT_Bn);  
  
    DDRB &= ~(1 << RC_RECEIVER_PORT);        // set receiver port as input
}


void OUT_Control(float position,uint8_t port){
    // this function switches port on depending on RC signal
    // position variable indicates 0%-100% stick position
    // ---------not used here but may be helpful 
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

 #ifdef Send_10K_CAL   // for the calibration oscillator
        ISR (TIMER0_COMPA_vect) {
         PORTB ^= 1 << PINB3;        // Invert pin PB3
        }
 #endif 
ISR(PCINT0_vect){
         // interrupt service routine for pin change interrupt to measure the RC servo pulse 
         // modified to use TCNT0 to allow delay to work.. 
    // if PINB is HIGH, a rising edge interrupt has happened
    // if PINB is LOW, a falling edge interrupt has happened
 
    if ( RISING_EDGE ){       // check if rising edge pin change interrupt (beginning of servo pulse)
   //   TCNT1 = 0;                           // reset counter (TCNT1 page 91 TCNT0 page 78
   //   TCCR1 = (1 << CS12) | (1 << CS11) | (1 << CS10);   // start timer1 with prescaler CK/64 --> 250 steps per 2ms (TCCR1 page 89
        TCNT0 = 0; 
        TCCR0B = (1 <<CS01) | (1<< CS00) ; // start timero ck/64 page 80 (256 count is 2.048ms .. ) so 1.5ms (mid) should be 187. 1ms should be 127 or so. 
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



void Stepper_Drive(int  in) {    // simple way to select 4 pin or 5 pin steppers etc. 
    if (HALF_STEP) {delayMicroseconds(StepSpeed);}else {delayMicroseconds(StepSpeed*2);}
    //Stepper5_Drive(in);  //(for 5 wire stepper drive such as found in educational kits with geared servo drives.)
    if (HALF_STEP) {Stepper4_half_step_Drive(in);} else {Stepper4_Drive(in); } 
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
//   Serial.print(in);
  /* stepper 4 phase changes
   *  expects 0 - 3 input
 *  A+ A- B+ B- 
 *  1  0  0  0   <
 *  0  0  1  0   up
 *  0  1  0  0   >
 *  0  0  0  1   down
 * becomes
   *  expects 0 - 3 input
 *  L(A+ A-) B+ B- 
 *  1  0  1  0   <up
 *  0  1  1  0   >upp
 *  0  1  0  1   >down 
 *  1  0  0  1   <down
 
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
  deadzone=GAIN*2.5;  // allow a deadzone for no response if only a small change is seen in the demand, to prevent "hunting" and allow drive to switch off
  aim=(pos*GAIN); // move this many steps per "unit" of Rc pulse width .. (range 0-125) 
  
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

void Move_To_ABS (int pos){  
  int diff,aim,deadzone; bool dir;
  deadzone=2;  
  aim=(pos); // No gains here! this code used only to move absolute to explore range of movement
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

void Count_Motor_Range(){ // Move motor absolute 40 steps to explore range. 
  for (int x=0;x<=40;x++){
    delay(1000); Move_To_ABS(x*40);}
    Move_To_ABS(0);
}
  

void IOTEST(){ // measure the RC sevo signal
  uint16_t Hystresis;
  if ( pulse_ready) {
         pulse_ready = 0;
 if ((count>=110)&&(count<=255)){  //  this bit could do with adjustment to accomodate the deliberate osccal changes? 
    if (Analog_mode){     
        
                   demand= int(count-110); //use 127*OSCNOM/100 ? Nominal range is approx 125 to 255 gives 0-130
                   PositionAchieved=false; }
 
             else {PositionAchieved=false;  // switched mode...add hystresis to avoid hunting Range Stepper_Position= 0 or Fullrange;
                                                   // for hystresis of 20
                                                   // add/sub  20*Stepper_Position/Fullrange 
                                                   Hystresis = 20*Stepper_Position/FullRange; 
                          if (count>=(180-Hystresis)){demand=FullRange;}
                                else {demand=0;} 
                          }
 } 

                   
         GIFR = (1 << PCIF);              // clear Pin Change Interrupt Flag  (datasheet page 52)
         GIMSK |= (1 << PCIE); }           // Pin Change Interrupt Enable (datasheet page 51)
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

void setup(){
  #ifdef Send_10K_CAL 
     Int_driven_10k();
  #else
   Analog_mode= false;
    #ifdef ANALOG
      Analog_mode= true;
    #endif
      
    Calibrate_OSCILLATOR();
    Init_PORT();
    Init_INTERRUPTS();
    Stepper_Position=2*FullRange; // A big number, must be larger than the actual number of max steps to ensure motor will reset to 0
    demand=0;
    PositionAchieved=false;
    //Serial.begin(115200); 
    Move_To_ABS(0);  //to hit endstop..;
   //calibration runs ?
    #ifdef Count_range_steps
         Count_Motor_Range();
    #endif
    
  #endif
  }

    
void loop(){
 
  #ifdef Send_10K_CAL 
    // will do (just) the 10k interrupt driven test tone.
  #else    
     if (!PositionAchieved) {  Move_To(demand);}
      IOTEST();
  #endif   
      
  }
