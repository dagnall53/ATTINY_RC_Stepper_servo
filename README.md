# ATTINY_RC_Stepper_servo
 ATTINY85 stepper motor drive with PWM (RC) control
 This code is a test to see if it is possible to build a ATTINY85 based "RC-servo" to use with miniature geared stepper motors. 
 
 it is based on "ATTINY85_RC_Receiver.c" (https://github.com/chiefenne/ATTINY85-RC-Receiver-Decoder/blob/master/main_low_resolution.c)
 
 But modified to use a different timer in the interrupt because I found that the one in the original seems to conflict with use of the standard "delay()".  - but I could be wrong!.
 
 Tested with 5 wire stepper and geared stepper as used in educational kits. and with 4 wire nano linear servos https://youtu.be/8g7uZkoPPww
 
 Program using "Digispark 8mhz No USB" option in Arduino. 
 
 Paramaters used give full movement on 4 wire nano linear servos.
  
 Stepper should have a mechanical end stop fitted, as the Setup code spins the motor in reverse during setup expecting to be physically stopped by the endstop. This sets (0). All movements are relative from the last position, so it is not suitable for repeated preciscion movement to a set position.
 
 I have added some test code in Setup that can be enabled to send out a 50kHz test tone. This can be used to adjust OSCCAL.. Just comment the code again before using the unit properly.
 
 If using cheap Chinese Difispark development boards, remember that P5 is not available and is a RESET!. 
 But apparently it can be used as an analog input - just do not go below the V(low) threshold (2.5v?) , or the chip will reset.
 
