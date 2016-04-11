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
from random import randint, randrange
import sys, os

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from ev3dev.auto import *

#Connect motors
rightMotor = LargeMotor(OUTPUT_A)
leftMotor  = LargeMotor(OUTPUT_D)
frontMotor = MediumMotor(OUTPUT_C)

# Connect sensors.
#ts1 = TouchSensor(INPUT_1);	assert ts1.connected
#ts4 = TouchSensor(INPUT_4);	assert ts4.connected
us = UltrasonicSensor(); assert us.connected
#gs = GyroSensor(); assert gs.connected
ls = ColorSensor(); assert ls.connected

#gs.mode = 'GYRO-ANG'

# We will need to check EV3 buttons state.
btn = Button()

# We need bools to determine current state.
backupState = False; #If a black line is detected on the light sensor, this is the backup state

# Define some contants that we shouldn't change in the program
CIRCLE_DIAMETER = 800
REDETECT_TIME = 2000
BLACK_LINE_VALUE = 18


def turn(dir, speed, runtime):
	"""
	Turn in the specified direction for a time
	at the specified speed.
	"""
	rightMotor.run_timed(duty_cycle_sp=-dir*speed, time_sp=runtime)
	leftMotor.run_timed(duty_cycle_sp=dir*speed, time_sp=runtime)

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

	
# Run the robot until a button is pressed.
#sleep(3000);
frontMotor.run_direct(duty_cycle_sp=75) # change duty cycle for more fun :)
forward(75,0,0)
while not btn.any():
        """
        Purpose of this is to test a mockup of the random direction sumo.
        """
        
        while not backupState and not btn.any():
                # Main sequence that will run until either backupState changes or a botton is pressed
                if ls.value() < BLACK_LINE_VALUE:
                        # Our light sensor detects we're on black, we need to backup now!
                        backupState = True
                
        if backupState:
                # If we exited main sequence due to a state change, complete the backup
                backup()
                # Turn a random direction
                randDir = randrange(-1,2,2)
                randTime = randint(250,1000)
                turn(randDir, 75, randTime)
                while any(m.state for m in (leftMotor, rightMotor)):
                        sleep((randTime/1000)) #seconds
                # Charge!
                forward(75,0,0)
                backupState = False

# Stop the motors before exiting.
rightMotor.stop()
leftMotor.stop()
frontMotor.stop()
