
 /*
  * Dagnall 2021 modified to use alternate Timer and drive stepper motor (I think this allows me to use  delay())..
  * Added code from http://becomingmaker.com/tuning-attiny-oscillator/ for interrupt based osc test frequency
  * 
  * /*
 * MODIFIED from  * ATTINY85_RC_Receiver.c
 * https://github.com/chiefenne/ATTINY85-RC-Receiver-Decoder
 * Created: 01.01.2019 20:06:29
 * Author : Andreas
 * Description: Read and interpret signals from RC Receivers
 
  * 
  * 
  * Program with Arduino: 
  * Attiny D Mellis board https://github.com/damellis/attiny
  *
  * Board:     ATtiny25/45/85
  * Processor: Attiny85
  * Clock    : Internal 8MHZ
  * 
  * for me : either: 
  * Programmer: Arduino as ISP 
  *     via an arduino programmer (Programmed using "ArduinoISP" from EXAMPLES)
  *    (NOTE- Burn Bootstrap the first time on the ATTINY85!)
  *     https://www.instructables.com/How-to-Program-an-Attiny85-From-an-Arduino-Uno/
  * 
  * Or: if using USBTinyISP, then add driver!.  (https://learn.adafruit.com/usbtinyisp/drivers)
  * 
  * 
  * 
  * 
  * set programmer to clock internal 8MHz
  * 
  * I believe this makes the comments about CKDIV* in the original code irrelevant.. 
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
//
//
// For motor drive pin outs see below.. I have used both FM116B (sot23 drivers) and also TB6612FNG.
 //
//
//
