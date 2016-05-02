#!/usr/bin/python

# -----------------------------------------------------------------------------
# Copyright (c) 2015 Denis Demidov <dennis.demidov@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# -----------------------------------------------------------------------------

from time   import sleep, time
import sys, os

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from ev3dev.auto import *

#Connect motors
rightMotor = LargeMotor(OUTPUT_A)
leftMotor  = LargeMotor(OUTPUT_D)
frontMotor = LargeMotor(OUTPUT_B) #CLAMP
sensorMotor = MediumMotor(OUTPUT_C) #SENSOR

# Connect sensors.
#ts1 = TouchSensor(INPUT_1);	assert ts1.connected
#ts4 = TouchSensor(INPUT_4);	assert ts4.connected
us = UltrasonicSensor(); assert us.connected
gs = GyroSensor(); assert gs.connected
ls = ColorSensor(); assert ls.connected #check name of light sensor class

gs.mode = 'GYRO-ANG'
#us.mode = 'US-DIST-CM'

# We will need to check EV3 buttons state.
btn = Button()

# We need bools to determine current state.
backupState = False; #If a black line is detected on the light sensor, this is the backup state
turnState = False; #If no robot is detected, this is the turn until found on ultrasonic state
moveState = False; #If ultrasonic is detected within a distance, this is the moving forward state

# Define some contants that we shouldn't change in the program
MIN_DIST = 100

def turn(dir, speed):
	"""
	Turn in the specified direction at the specified speed. Doesn't stop until told to stop.
	"""
	rightMotor.run_direct(duty_cycle_sp=-dir*speed)
	leftMotor.run_direct(duty_cycle_sp=dir*speed)

def rotate(dir, speed):
	"""
	Turns the sensor in the direction, dir, at a specific speed
	"""
	sensorMotor.run_direct(duty_cycle_sp=dir*speed)

def forward(speed, bias, biasDir):
	"""
	Start both motors. `run-direct` command will allow to vary motor
	performance on the fly by adjusting `duty_cycle_sp` attribute.
	Doesn't stop until told to stop.
	Bias determines if there is a bias on one motor to move faster for correction.
	The bias is simply added to the speed for the current biasDirection
	"""
	# todo: check directions for me please
	if biasDir == 1:
                rightMotor.run_direct(duty_cycle_sp=speed+bias)
                leftMotor.run_direct(duty_cycle_sp=speed)
        elif biasDir == -1:
                rightMotor.run_direct(duty_cycle_sp=speed)
                leftMotor.run_direct(duty_cycle_sp=speed+bias)
        else:
                rightMotor.run_direct(duty_cycle_sp=speed)
                leftMotor.run_direct(duty_cycle_sp=speed)

def stop():
	"""
	Stop both motors.
	"""
    rightMotor.stop(stop_command='brake')
	leftMotor.stop(stop_command='brake')

def backup():
	"""
	#Back away from an obstacle.
	"""

	# Sound backup alarm.
	#Sound.tone([(1000, 500, 500)] * 3)

	# Turn backup lights on:
	Leds.set_color(Leds.RIGHT, Leds.RED)
	Leds.set_color(Leds.LEFT, Leds.RED)

	# Stop both motors and reverse for 1.5 seconds.
	# `run-timed` command will return immediately, so we will have to wait
	# until both motors are stopped before continuing.
	stop()
	rightMotor.run_timed(duty_cycle_sp=-75, time_sp=750)
	leftMotor.run_timed(duty_cycle_sp=-75, time_sp=750)

	# When motor is stopped, its `state` attribute returns empty list.
	# Wait until both motors are stopped:
	while any(m.state for m in (leftMotor, rightMotor)):
		sleep(0.1)

	# Turn backup lights off:
	Leds.set_color(Leds.RIGHT, Leds.GREEN)
	Leds.set_color(Leds.LEFT, Leds.GREEN)

def hasRight(gs_val):
	"""
	Checks if there is a path on the right
	"""
	if gs_val > MIN_DIST:
		return True
	else:
		return False
	
def hasFront(gs_val):
	"""
	Checks if there is a path on the front
	"""
	if gs_val > MIN_DIST:
		return True
	else:
		return False

def detectRight():
	"""
	Returns the distance value of the right 
	"""
	while not gs.value() % 90 == 0:
		sensorMotor.rotate(1, 50)

	right_dist = us.value
	return right_dist

def detectFront():
	"""
	Returns the distance value of the front
	"""
	while not gs.value() % 90 == 0:
		sensorMotor.rotate(-1, 50)

	front_dist = us.value
	return front_dist	 

# Run the robot until a button is pressed.
while not btn.any():        	
	"""
	FRONT| RIGHT| MOVE
	---------------------
	TRUE   FALSE  FORWARD
	FALSE  TRUE   TURN RIGHT
	TRUE   TRUE   TURN RIGHT 
	FALSE  FALSE  TURN RIGHT
	"""
	right_dist = detectRight()

	front_dist = detectFront()

	if hasFront(front_dist) and not hasRight(right_dist): #MOVE FORWARD
		while detectFront() >= MIN_DIST or not hasRight(right_dist):
			forward(50, 0, 0)
	elif not hasFront(front_dist) and hasRight(right_dist): #MOVE RIGHT
		while not gs.value() % 90 == 0:
			turn(1, 50)
		while detectFront() >= MIN_DIST:
			forward(50, 0, 0)
	elif hasFront(front_dist) and hasRight(right_dist): #Choice of forward & right, will turn RIGHT
		while not gs.value() % 90 == 0:
			turn(1, 50)
		while detectFront() >= MIN_DIST or not hasRight(right_dist):
			forward(50, 0, 0)
	elif not hasFront(front_dist) and not hasRight(right_dist): #Rotate 180 degrees and move forward again 
		while not gs.value() % 180 == 0:
			turn(1, 50)
		while detectFront() >= MIN_DIST or not hasRight(right_dist):
			forward(50, 0, 0)

# Stop the motors before exiting.
rightMotor.stop()
leftMotor.stop()
sensorMotor.stop()