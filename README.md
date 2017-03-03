# csulbusli2017

##BASIC DESCRIPTION:

Note: all this assumes the raspberry pi computer vision thing is totally separate.    
Also, the code in Controller_Nonlinear_01.ino is incomplete! still needs:

	-computed controller gains (Ki)
	-parameters for servo and fan, to keep performance up to scratch

###SETUP
	-initialize instruments: gps and altimeter
	-initialize actuators: esc and servo
	-read instruments: gps and altimeter upon reset, store in variables.
	-light status lights to verify functionality of instruments

###LOOP

	-conditional test: what part of the mission are we in?
		1. Pre-deployment: don't do anything
		2. Deployment: wait a while for parafoil to deploy properly
		4. Control: Do all of the following on loop:
			-read instruments
			-convert sensor data to xyz coordinates
			-filter data if necessary (IIR filter function included)
			-calculate distance squared (faster than calculating square root)
			-calculate rate of descent (with IIR derivative filter)
			-calculate control effort: see "CONTROLLER"
	-write appropriate control signals to stepper, servo and fan
---
###CONTROLLER

A feedback controller attempts to get the current state to a desired state.    
It does so by operating on **error signals**. Various error signals are calculated:
	
	-distance error -- difference btwn desired and actual distance to a target
	-bearing error -- " " " " " angle over ground, or bearing angle
	-descent rate error -- " " " " " descent rate

The error signal being minimized will depend on:
	
	-desired flight characteristics
	-how far the glider is from the launch site

The controller presupposes a **system model** which emulates reality in some way.  
The elements of the system are modeled like so:

	-servo steering mechanism with respect to turning radius:
		-input: desired turning radius (pwm signal)
		-output: well defined turning radius
		-response: first order (r(t) = 1-exp(-at))
	-servo steering mechanism with respect to angle over ground:
		-input: desired angle over ground (pwm signal)
		-output: some angle over ground (bearing)
		-response: second order -- integral of r(t)
	-fan propulsion/pitch control subsystem:
		-input: desired descent rate (pwm signal)
		-output: well defined pitch and descent rate
		-response: also first order
	-The coordinate system: two different models for different situations:
		1. xy plane broken into 8 45 degree pie slices with target at the center
		2. xy location reduced to a one dimensional distance from a target

These are the scenarios I have considered, and how I think they can be dealt with:
	
	1. The glider deploys much more than 300ft from the target:
		-position mapped to one of 8 pie slice regions
		-region determines desired angle over ground
		-calculate bearing error
		-drive servo based on error: servo angle = K*(bearing error)
	2. The glider is within 300 ft of launch site:
		-position is exemplified by the distance from the target
		-calcualte distance error
		-conditional test:
			-if within X ft of target, servo angle set to zero
			-if within Y ft of target, control servo angle like: servo angle = K*(distance error) 
			-if very far from target again, do control 1. again.
		-NOTE: decide on whether you want to make left or right turns
	3. A good amount of time must be spent scanning the area while taking pictures:
		-decide on flightpath or some quality of a desired flightpath
		-use the 8 regions to calculate alternate desired bearing angles
			-my simplest idea is just to spiral inward, or outward

Altitude control will be happening independently. It will be accomplished like this:
	
	-calculate descent rate error
	-apply actuation to fan based on this: fan speed = K*(descent rate error)

**NOTE**: I did not do any fancy compensation: just proportional control. However, with the    
IIR filter I included, any classical control compensator is easily realizable -- see the    
Matlab script I put in this repo. The implementation would replace something like    
fan speed = K*(descent rate error) with fan speed = iir2(descent rate error, buff, a, b)

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