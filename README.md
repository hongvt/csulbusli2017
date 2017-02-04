# csulbusli2017

Manual override subsystem description found here: --> https://jeffreyscomputer.github.io/PWcomparator.html
This completely bypasses MCU with a simple external circuit.

I am writing the Arduino sketch for the board Pro Mini,  because that is all I have on hand. If it needs to be changed, it won't be hard.

BASIC DESCRIPTION:

SETUP:
-read gps and altimeter upon reset, store in variables.
-light status lights to verify functionality of instruments
-set up pwm outputs

LOOP:
-read gps and altimeter
    -->need to figure out best way to do this -- the GPS has altitude data -- must investigate this.
-convert sensor data to xyz coordinates
-filter data if necessary (IIR filter function included)
-calculate distance squared (faster to calculate than a square root)
-calculate rate of descent (with IIR derivative filter)

-2 separate feedback controllers control the distance squared from the launch site and the rate of descent quasi independently. Output of the controller is the pulse widths to be applied to the servo and fan.

-write pwm to servo and fan


CONSIDERATIONS:
GPS limits the speed -- max sample rate is 10Hz, and max rate of gps fix is 5Hz. 
