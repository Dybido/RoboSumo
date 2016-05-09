#!/usr/bin/python

#Possible loops.
#Possible 4 way intersections
#Probably right angled walls.

_DISTANCE_BETWEEN_WALLS = 290 #mm
_WALL_HEIGHT = 29 #cm
_ERROR_BOUNDS_GS = 5 #degrees
_ERROR_BOUNDS_US = 2 #cm
_STANDARD_SPEED = 40 #duty cycle

from time   import sleep
import termios, fcntl, sys, os
fd = sys.stdin.fileno()

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from ev3dev.auto import *

#Connect motors
rightMotor = LargeMotor(OUTPUT_A)
leftMotor = LargeMotor(OUTPUT_B)
sensorMotor = MediumMotor(OUTPUT_C) #SENSOR

# Connect touch sensors.
us = UltrasonicSensor(); assert us.connected
ls = ColorSensor(); assert ls.connected
gs = GyroSensor(); assert gs.connected

gs.mode = 'GYRO-ANG'

# We will need to check EV3 buttons state.
btn = Button()

#intersection



#Simply to mod gyro value
def gyroValue():
	return gs.value()%360

#Function to reset the pivot on the top of the robot
#Must have an angle to reset to
def resetPivot(resetAngle):
	pass

#Function to move forward continuously
#Requires speed for both wheels
def moveFoward(lft, rgt):
	rightMotor.run_direct(duty_cycle_sp=rgt)
	leftMotor.run_direct(duty_cycle_sp=lft)
	
#Monitors and corrects the forward movement to stay on the same forward direction
#Requires a direction relative to the beginning of the program and speed to adhere to
def forwardMonitor(forwardAngle, speed):
	if(gyroValue() < forwardAngle):
		moveFoward(speed, (speed/5)*4)
	elif(gyroValue() > forwardAngle):
		moveFoward((speed/5)*4, speed)

#Function to turn a specific angle
#Must have an angle in degrees, a valid gyro and speed
def moveTurn(angle, speed, motorList):
	global forwardAngle
	forwardAngle += angle
	forwardAngle = forwardAngle%360
	currentAngle = gyroValue()
	print "current angle:", currentAngle
	finishedAngle = (currentAngle + angle)%360
	#turn until we get to the angle we want
	while(gyroValue() < finishedAngle - _ERROR_BOUNDS_GS or gyroValue() > finishedAngle + _ERROR_BOUNDS_GS):
		#TODO check logic
		print gyroValue(), " ", finishedAngle
		for motor in motorList:
			if(angle < 0):
				#TODO check direction
				motor.run_direct(duty_cycle_sp=speed*-1)
			elif (angle > 0):
				#TODO check direction
				motor.run_direct(duty_cycle_sp=speed*1)
	print "finished angle", gyroValue()
	for motor in motorList:
		motor.stop()
	
	
def sensorThread():
	"""Checks values on each side for intersections"""
	#TODO: sensor class to store values? or join
	hasLeft = hasNoWall(detectDistance(-90))
	hasFront = hasNoWall(detectDistance(0))
	hasRight = hasNoWall(detectDistance(90))

def hasNoWall(gs_val):
	"""
	Checks if there is a path on a side
	"""
	if gs_val > _DISTANCE_BETWEEN_WALLS:
		return True
	else:
		return False
	

def detectDistance(abs_angle):
	"""
	Returns the distance value of the given angle 
	"""
	sensorMotor.run_to_abs_pos(position_sp=-abs_angle, stop_command="brake")
	while any(m.state for m in ([sensorMotor])):
		sleep(0.1)
	sleep(0.2)
	left_dist = us.value()
	return left_dist
	
			
#Print sensors		
def printSensors():
	print "Gyro ", gs.value()
	print "Ultra ", us.value()
	print "Light ", ls.value()
	
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)

oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

forwardAngle = gyroValue() #allow us to monitor the path as we are moving forward
sensorMotor.duty_cycle_sp = 50
rightMotor.duty_cycle_sp = 50
leftMotor.duty_cycle_sp = 50
try:
	print "ready!";
	while not btn.any():
		"""
		Run a thread for detecting left front and right.
		Join thread to be used by main thread which controls direction and motors
		
		Motors: Motor.count_per_rot, run_to_rel_pos
		
		How to keep going forward:
		Each turn add angle and mod 360. All angles will be 0-360
		
		currentDir = gyroValue()
		if(forwardAngle + degrees > 360 || forwardAngle - degrees < 0) {
			//shift angles to work at a better range
			changedForwardAngle = (forwardAngle-180)%360
			changedCurrentDir = (currentDir-180)%360
			
			if(changedCurrentDir - degrees) > (changedForwardAngle):
				# print('right')
				rightMotor.duty_cycle_sp = 40
			TODO:Finish
		}
		
		"""
		
		#Run sensorThread()

		#Get values out of sensorThread specifying if theres walls or not
		if(((hasLeft or hasRight) and hasFront) or (hasLeft and hasRight)):
			#Create a new intersection node
			#Inside the node store our travel angle and other data...
			
			#Make a decision on direction and add to nodes, turn etc... yeah
			
		else:
			moveFoward(47,47)
			direction = gyroValue();
			print "gyrovalue: ", (direction-5)%360, forwardAngle
			
			forwardAngle-=90
			#moveTurn(90, 50, [leftMotor, rightMotor])
			leftMotor.run_to_rel_pos(position_sp=225, stop_command="brake")
			rightMotor.run_to_rel_pos(position_sp=-225, stop_command="brake")
			while any(m.state for m in (leftMotor, rightMotor)):
				sleep(0.1)
			moveFoward(47,47)
			sleep(2)
		try:
			c = sys.stdin.read(1)
			print "Current char", repr(c)
			#The function tester!
			if(c == 'c'):
				Sound.tone([(1000, 500, 500)] * 3)
				rightMotor.stop()
				leftMotor.stop()
				sensorMotor.stop()
				break
				
		#printSensors()"""
			
		except IOError: pass
finally:
	termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
	fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

rightMotor.stop()
leftMotor.stop()
sensorMotor.stop()