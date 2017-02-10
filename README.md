# csulbusli2017

##BASIC DESCRIPTION:

Note: all this assumes the raspberry pi computer vision thing is totally separate.

###SETUP
	-initialize stepper, esc and servo
	-read instruments: gps and altimeter upon reset, store in variables.
	-light status lights to verify functionality of instruments

###LOOP

	-conditional test: what part of the mission are we in?
		1. Pre-deployment: just keep the nose cone closed.
		2. Deployment: Step the stepper open until limit switch closes.
		3. Wait: give the parafoil a few seconds to deploy properly
		4. Control: Do all of the following on loop:
			-read instruments
			-convert sensor data to xyz coordinates
			-filter data if necessary (IIR filter function included)
			-calculate distance squared (faster than calculating square root)
			-calculate rate of descent (with IIR derivative filter)
			-calculate control effort: see "CONTROLLER"
			-write pwm to servo and fan
---
###CONTROLLER

A feedback controller attempts to get the current state to a desired state.    
It does so by operating on **error signals**. Various error signals are calculated:
	-**distance error** -- difference btwn desired and actual distance to a target
	-**bearing error** -- " " " " " angle over ground, or bearing angle
	-**descent rate error** -- " " " " " descent rate
The error signal being minimized will depend on:
	-desired flight characteristics
	-how far the glider is from the launch site

These are the scenarios I have considered, and how I think they can be dealt with:
	1. The glider deploys much more than 300ft from the target
		-
Space around launch site divided into 8 45 degree pie shaped regions--region number determines desired angle over ground.      
	2. Once near launch site, controller will act to maintain a predefined distance from a target.
	3. Altitude control will be separate, and determined by the time.

---

##CONSIDERATIONS:

	-GPS limits the speed -- max sample rate is 10Hz, and max rate of gps fix is 5Hz.

##MANUAL OVERRIDE SUBSYSTEM:
description found here:   
--> https://jeffreyscomputer.github.io/PWcomparator.html

This completely bypasses MCU with a simple external circuit.  

##LIBRARIES:

	-Servo.h  
	-Adafruit_GPS.h  
	-Wire.h     ----> I believe the barometer is I2c


##ALSO:
anyone feel free to fancy up the code when I'm done. I'm not very sophisticated.
