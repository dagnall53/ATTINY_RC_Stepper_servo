# ATTINY_RC_Stepper_servo
 ATTINY85 stepper motor drive with PWM (RC) control or with simple (on / off).
 
 Designed to make it easy to use "RC-servo" control with Inexpensive but powerful miniature geared stepper motors. 
 Video here https://youtu.be/CbauUchePao
 
 This code is for reasonably experienced users, who are happy to modify and "play".
 I used it initialy as a "PWM detector" (RC) that could switch between two positions. (as in the video).
 I also tried it as a "servo" where the position was proportional to the PWM (RC) signal. 

Both these "Servo" (RC - PWM input) options require that the device can reasonably accurately measure the input pulse width. 
Ideally each device then needs to be calibrated to set the internal clock. I have provided a few bits of code to help this calibration, but this is for experienced programmers and experimenters only.

The "NON SERVO" operation does not require precision internal timing, and I am now finding this more useful. But it is just a two position result. - "On or Off"
But you can have quite complex movements programmed for each "position".  
My default uses " Backoff" - where I move to a position expected to be mechanically limited, and then "backoff" a little to release the pressure on the mechanism.
With suitable mechanisms, this has proved very useful.  -- But the code is there to be adapted!. ---     
 
The Code is based on "ATTINY85_RC_Receiver.c" (https://github.com/chiefenne/ATTINY85-RC-Receiver-Decoder/blob/master/main_low_resolution.c)
 but modified to use a different timer in the interrupt because I found that the one in the original seems to conflict with use of the standard "delay()".  (- but I could be wrong!.)
 
 As saved, the Hardware expects: 
 PB0 is the input(Servo type PWM) (or Binary 0/1)  signal
 PB1/PB2 are the "A" motor coil drivers
 PB3/PB4 are the "B" motor coil drivers.
 
 I have a LED connected to PB1 to show activity.
 The code can sense if PB5 is "connected to ground" by a 100k to switch between modes. This is used in the setup function. 
 
Originally it was tested with the 5 wire stepper motors as used in educational kits. 
I have retained the 5 wire drive code, so you can modify to use this if you need. 

There are LOTS of options to play with.. I have tried to make them optional by using # defines. 

I have programmed using Arduino using both 
: ATtiny Microprocessors : (https://github.com/damellis/attiny) "Attiny25/45/85" with "Attiny85" Clock internal 8Mhz, programmer USBTinyISP     
: Digistump AVR Boards: (https://github.com/SpenceKonde/ATTinyCore) "Digispark 8mhz No USB" . using the "Microneucleus" programer. (This is useful for the Digistump (USB type) boards.) 

I found that occasionally the Digistump / microneucleus route would crash, leaving  "bricked" hardware that would not accept a new program. 
But the USBTinyISP programmer route using board "ATtiny Microprocessors" does not need the microneucleus, so I have found it more reliable, and "recovered" the Digistump hardware when it got corrupted.

NOTE: the "clone" Digistump boards differ from the real digstump boards in the use of PB5  (rst) I have ONLY tested with bare chips and "Clones".. 

The code uses PB5 (RST) as an "analog" pin on startup. Connecting an optional 100k to ground should be detected, and this allows simple switching between Pseudo Analog "proportional servo" operation, and "end to end" operation. - But of course this can be altered simply by modifying the code.

End to end operation will move the drive to its end-stop when the PWM input exceeds a threshold, and return to the other end-stop when the input reduces below the threshold. (including some built in hystresis).

The saved parameters should give give full movement on the micro linear 4 wire geared stepper motor drives.
  
It is very important that the drive have at least a zero position mechanical end stop fitted, as the setup code spins the motor in reverse during setup expecting to be physically stopped by the endstop. This sets position (0). All movements are then relative from the last position, 

The Attiny 85 internal oscillator is not known for its accuracy, and some codes require it to be individually calibrated. This may be necessary if it is important that say 1ms input pulse is exactly "90 degrees" on the servo. I have added some (commented out) test code that can be enabled to send out a test tones. This can be used to adjust OSCCAL.. Just comment the code again before using the unit properly.

HOWEVER: when used as an "end to end" drive, switching at a "threshold" PWM input, the actual "threshold" PWM setting is less important, and I would recommend using this mode for most miniature drive applications if possible.

A basic circuit is shown in "Basic-Circuit" in the PCB sub folder.  
I used Easy EDA and had some prototypes made for the tests. (before the Chip shortages!!)   

