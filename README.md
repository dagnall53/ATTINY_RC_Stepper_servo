# ATTINY_RC_Stepper_servo
 ATTINY85 stepper motor drive with PWM (RC) control
 This code is a test to see if it is possible to build a ATTINY85 based "RC-servo" to use with miniature geared stepper motors. 
 
 it is bsed on "ATTINY85_RC_Receiver.c" (https://github.com/chiefenne/ATTINY85-RC-Receiver-Decoder/blob/master/main_low_resolution.c)
 
 But modified to use a different timer in the interrupt because I found that the one in the original seems to conflict with use of the standard "delay()".  - but I could be wrong!.
 
 Tested with 5 wire stepper and geared stepper as used in educational kits.
 
 Program using "Digispark 8mhz No USB" option in Arduino. 
 
 Paramaters used give about 0-240 degrees of control.
 
 Stepper should have a mechanical end stop fitted, as the Setup code spins the motor in reverse expecting to be physically stopped by the endstop. 
 
 If using cheap Chinese Difispark development boards, remember that P5 is not available and is a RESET!. 
 But apparently it can be used as an analog input - just do not go below the V(low) threshold (2.5v?) , or the chip will reset.
 
