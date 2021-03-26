# ATTINY_RC_Stepper_servo
 ATTINY85 stepper motor drive with PWM (RC) control
 Designed to make it easy to use "RC-servo" control with Inexpensive but powerful miniature geared stepper motors. 
 Video here https://youtu.be/CbauUchePao
 
 it is based on "ATTINY85_RC_Receiver.c" (https://github.com/chiefenne/ATTINY85-RC-Receiver-Decoder/blob/master/main_low_resolution.c) but modified to use a different timer in the interrupt because I found that the one in the original seems to conflict with use of the standard "delay()".  (- but I could be wrong!.)
 
 As saved, the Hardware expects: 
 PB0 is the input(Servo type PWM)  signal
 PB1/PB2 are the "A" motor coil
 PB3/PB4 are the "B" motor coil.
 
Originally it was tested with the 5 wire stepper motors as used in educational kits. I have retained the 5 wire drive code, so you can modify to use this if you need. 

I have programmed using Arduino using both 
: ATtiny Microproecessors : "Attiny25/45/85" with "Attiny85" Clock internal 8Mhz, programmer USBTinyISP and    
: Digistump AVR Boards "Digispark 8mhz No USB" . using the "Microneucleus" programer. (This is useful for the Digistump (USB type) boards.) 

I found that occasionally the Digistump / microneucleus route would crash, leaving  "bricked" hardware that would not accept a new program. But the USBTinyISP programmer route using board "ATtiny Microprocessorss" does not need the mrconeucleus, so I have found it more reliable, and "recovered" the Digistump hardware when it got corrupted.

NOTE: the "clone" Digistump boards differ from the real digstump boards in the use of PB5  (rst) I have ONLY tested with bare chips and "Clones".. 

The code uses PB5 (RST) as an analog pin on startup. Connecting an optional 100k to ground should be detected, and this allows simple switching between Pseudo Analog "proportional servo" operation, and "end to end" operation. - But of course this can be altered simply by modifying the code.

End to end operation will move the drive to its endstop when the PWM input exceeds a threshold, and return to the other endstop when the input reduces below the threshold. (including some built in hystresis).

The saved paramaters ushould give give full movement on the micro linear 4 wire geared stepper motor drives.
  
It is very important that the drive have at least a zero position mechanical end stop fitted, as the setup code spins the motor in reverse during setup expecting to be physically stopped by the endstop. This sets position (0). All movements are then relative from the last position, 

The Attiny 85 internal oscillator is not knwon for its accuracy, and some codes require it to be individually calibrated. This may be necessary if it is important that say 1ms input pulse is exactly "90 degrees" on the servo. I have added some (commented out) test code that can be enabled to send out a test tones. This can be used to adjust OSCCAL.. Just comment the code again before using the unit properly.

HOWEVER: when used as an "end to end" drive, switching at a "threshold" Pwm input, the actual "threshold" PWM setting is less important, and I would recommend using this mode for most miniature drive applications if possible.

I have designed a small PCB to make this easy to use. This can be obtained from PCB way or other suppliers such as JLPCB. 
 
Adjusted folder organization