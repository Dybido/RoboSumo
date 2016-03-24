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

# In this demo an Explor3r robot with touch sensor attachement drives
# autonomously. It drives forward until an obstacle is bumped (determined by
# the touch sensor), then turns in a random direction and continues. The robot
# slows down when it senses obstacle ahead (with the infrared sensor).
#
# The program may be stopped by pressing any button on the brick.
#
# This demonstrates usage of motors, sound, sensors, buttons, and leds.

from time   import sleep
import sys, os

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from ev3dev.auto import *

#Connect motors
rightMotor = LargeMotor(OUTPUT_A)
leftMotor  = LargeMotor(OUTPUT_D)

# Connect touch sensors.
ts1 = TouchSensor(INPUT_1);	assert ts1.connected
ts4 = TouchSensor(INPUT_4);	assert ts4.connected
us = UltrasonicSensor(); assert us.connected
gs = GyroSensor(); assert gs.connected

gs.mode = 'GYRO-ANG'

# We will need to check EV3 buttons state.
btn = Button()

def turn(dir, speed):
	"""
	Turn in the specified direction at the specified speed.
	"""
	# We want to turn the robot wheels in opposite directions
	rightMotor.run_direct(duty_cycle_sp=-dir*speed)
	leftMotor.run_direct(duty_cycle_sp=dir*speed)

def forward(speed):
	"""
	Start both motors. `run-direct` command will allow to vary motor
	performance on the fly by adjusting `duty_cycle_sp` attribute.
	"""
	rightMotor.run_direct(duty_cycle_sp=speed)
	leftMotor.run_direct(duty_cycle_sp=speed)

def stop():
        """
        Stop both motors.
        """
        rightMotor.stop(stop_command='brake')
	leftMotor.stop(stop_command='brake')

"""def backup():
	"""
	Back away from an obstacle.
	"""

	# Sound backup alarm.
	Sound.tone([(1000, 500, 500)] * 3)
	#Sound.speak('Shit shit shit shit')

	# Turn backup lights on:
	Leds.set_color(Leds.RIGHT, Leds.RED)
	Leds.set_color(Leds.LEFT, Leds.RED)

	# Stop both motors and reverse for 1.5 seconds.
	# `run-timed` command will return immediately, so we will have to wait
	# until both motors are stopped before continuing.
	rightMotor.stop(stop_command='brake')
	leftMotor.stop(stop_command='brake')
	rightMotor.run_timed(duty_cycle_sp=-50, time_sp=1500)
	leftMotor.run_timed(duty_cycle_sp=-50, time_sp=1500)

	# When motor is stopped, its `state` attribute returns empty list.
	# Wait until both motors are stopped:
	while any(m.state for m in (leftMotor, rightMotor)):
		sleep(0.1)

	# Turn backup lights off:
	Leds.set_color(Leds.RIGHT, Leds.GREEN)
	Leds.set_color(Leds.LEFT, Leds.GREEN)
"""
	
# Run the robot until a button is pressed.
start()
while not btn.any():

	"""
        # If we bump an obstacle, back away, turn and go in other direction.

	if ts1.value():
		backup()
		turn(1)
		start()

	if ts4.value():
		backup()
		turn(-1)
		start()

	# Keep the robot going in the same direction

	direction = gs.value();
	# print direction

	if direction > 5:
		# print('right')
		rightMotor.duty_cycle_sp = 5
	elif direction < -5:
		# print('left')
		leftMotor.duty_cycle_sp = 5
	else:
		leftMotor.duty_cycle_sp = 75
		rightMotor.duty_cycle_sp = 75

	# Ultrasonic sensor will measure distance
	# to the closest object in front of it.
	distance = us.value();

	if distance > 300:
		# Path is clear, run at full speed.
		dc = 75
	else:
		# Obstacle ahead, slow down.
		dc = 30

	for m in (leftMotor, rightMotor):
		m.duty_cycle_sp = dc

	print rightMotor.position, leftMotor.position
        """
# Stop the motors before exiting.
rightMotor.stop()
leftMotor.stop()
