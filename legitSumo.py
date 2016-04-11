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
frontMotor = MediumMotor(OUTPUT_C)

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
ROBOT_CHASE_DISTANCE = 500 #mm
REDETECT_TIME = 2000 #milliseconds
TURN_TIME = 2000
BLACK_LINE_VALUE = 20 #ls reflected light


def turn(dir, speed):
	"""
	Turn in the specified direction at the specified speed. Doesn't stop until told to stop.
	"""
	rightMotor.run_direct(duty_cycle_sp=-dir*speed)
	leftMotor.run_direct(duty_cycle_sp=dir*speed)

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
sleep(3)
frontMotor.run_direct(duty_cycle_sp=75) # change duty cycle for more fun :)
while not btn.any():
        """
        Backup overrides other objectives and runs sequentially.
        The loop doesn't run sequentially asuch as it allows for continual state
        checking while the motors are running.
        States -
        moveState is set when a robot is detected. It is unset once we've
        lost the robot for too long.
        turnState is when when there is no robot detected. It is unset as soon as
        a robot is detected.
        This allows for three different possibilities, either we don't know of the robot
        and are turning, we know of the robot and are moving forward, or we have only just
        lost the robot (within the time set by REDETECT_TIME) and want to adjust our movement to hopefully rehit it.

        We also have a backupState which override everything, and this is only called if we have about to pass over a line
        """
        moveAngle = 0
        moveTime = 0
        turnTime = 0
        stateChange = False #todo: test if we need this. Program SHOULD work without it,
                            #but nonetheless, the motors might bug out if forward/turn is contantly called??
        turnState = True # Initially we want to go into a turning state
        moveState = False
        
        while not backupState and not btn.any():
                # Main sequence that will run until either backupState changes or a botton is pressed
                #print 'us: ', us.value()
                
                # First lets determine if we need to make state changes
                if us.value() < ROBOT_CHASE_DISTANCE:
                        # Robot detected
                        moveState = True
                        turnState = False
                        # Record some values for later use of when we first saw the robot
                        moveAngle = gs.value() # Record gyro angle for sick trig/checking
                        moveTime = time()*1000 + REDETECT_TIME # Record some sort of time... to start our full rotation again
                else:
                        # Robot has been lost OR robot simply not detected
                        # In our action we'll have to determine what's happening
                        if(turnState == False):
                               turnTime = time()*1000 + TURN_TIME
                        if(time()*1000 > turnTime):
                                moveState = True
                        turnState = True
                        

                if time()*1000 > moveTime:
                        # Our time to redetect has lapsed, go to pure detection mode
                        moveState = False

                if ls.value() < BLACK_LINE_VALUE:
                        # Our light sensor detects we're on black, we need to backup now!
                        backupState = True


                # Do our actions
                if moveState & turnState:
                        # This should mean we've just lost the robot,
                        # the best case is to hope to redetect by turning in the same? or opposite? direction we turned before
                        # Currently this is set to move slightly in the opposite direction
                        if moveAngle < gs.value(): # todo: check directions here again
                                forward(50,25,1)
                        else:
                                forward(50,25,-1)
                elif moveState:
                        # Should mean we've detect the robot, charge forward!
                        forward(75,0,0)
                elif turnState:
                        # Should mean robot hasn't been detected and we've gone past any attempted redetection time
                        # turn in an anticlockwise? todo: figure out if there is a better way to judge which way to turn
                        turn(1, 50) #todo: check this whole thing and its direction
                else:
                        # Neither state is set, this should never happen
                        print "What hath happened"

        if backupState:
                # If we exited main sequence due to a state change, complete the backup
                backup()
                backupState = False

# Stop the motors before exiting.
rightMotor.stop()
leftMotor.stop()
frontMotor.stop()
