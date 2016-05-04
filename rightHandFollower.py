#!/usr/bin/python

#Possible loops.
#Possible 4 way intersections
#Probably right angled walls.

_DISTANCE_BETWEEN_WALLS = 20 #cm
_WALL_HEIGHT = 29 #cm
_ERROR_BOUNDS_GS = 3 #degrees
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
#ls = ColorSensor(); assert ls.connected
gs = GyroSensor(); assert gs.connected

gs.mode = 'GYRO-ANG'

# We will need to check EV3 buttons state.
btn = Button()

#Simply to mod gyro value
def gyroValue():
	return gs.value()%360

startAngle = gyroValue() #Our initial angle
totalError = 0
#Sets the diference between angles to a total additions list.
def setTurnRecalibration(startAngle, finishedAngle):
	global totalError
	errorAngle = startAngle - finishedAngle
	totalError += errorAngle

#Retrieves additions list
def getTurnRecalibration():
	global totalError
	temp = totalError
	totalError = 0
	return temp
	
	

#Function to move forward a specific distance
#Must be able to access ultrasonic
def stepForward(distance, speed):
	currentDist = us.value()
	finishedDist = currentDist - distance
	#move until we get to the distance we want
	while(us.value() < finishedDist - _ERROR_BOUNDS_US or us.value() > finishedDist + _ERROR_BOUNDS_GS):
		if(us.value() < finishedDist):
			#We want to move back because we've overshot
			moveFoward(-speed,-speed)
		elif(us.value() > finishedDist):
			#move forward until we're in bounds
			moveFoward(speed,speed)

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
		moveFoward(60, 50)
	elif(gyroValue() > forwardAngle):
		moveFoward(50, 60)

#Function to turn a specific angle
#Must have an angle in degrees, a valid gyro and speed
def moveTurn(angle, speed, motorList):
	currentAngle = gyroValue()
	print "current angle:", currentAngle
	finishedAngle = (currentAngle + angle + getTurnRecalibration())%360
	#turn until we get to the angle we want
	while(gyroValue() < finishedAngle - _ERROR_BOUNDS_GS or gyroValue() > finishedAngle + _ERROR_BOUNDS_GS):
		#TODO check logic
		print gyroValue(), " ", finishedAngle
		for motor in motorList:
			if(angle < 0):
				#TODO check direction
				motor.run_direct(duty_cycle_sp=speed*1)
			elif (angle > 0):
				#TODO check direction
				motor.run_direct(duty_cycle_sp=speed*-1)
	print "finished angle", gyroValue()
	setTurnRecalibration(finishedAngle, gyroValue()) #Set the error in our turning.
	for motor in motorList:
		motor.stop()
	

def hasRight(gs_val):
	"""
	Checks if there is a path on the right
	"""
	if gs_val > _DISTANCE_BETWEEN_WALLS:
		return True
	else:
		return False
	
def hasFront(gs_val):
	"""
	Checks if there is a path on the front
	"""
	if gs_val > _DISTANCE_BETWEEN_WALLS:
		return True
	else:
		return False

def detectRight():
	"""
	Returns the distance value of the right 
	"""
	moveTurn(90, 50, [sensorMotor])

	right_dist = us.value()
	return right_dist

def detectFront():
	"""
	Returns the distance value of the front
	"""
	moveTurn(-90, 50, [sensorMotor])

	front_dist = us.value()
	return front_dist
	
			
#Print sensors		
def printSensors():
	print "Gyro ", gs.value
	print "Ultra ", us.value
	print "Light ", ls.value
	
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)

oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

forwardAngle = gyroValue() #allow us to monitor the path as we are moving forward
try:
	print "ready!";
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
			#while detectFront() >= MIN_DIST or not hasRight(right_dist):
			forwardMonitor(forwardAngle%360, 50)
		elif not hasFront(front_dist) and hasRight(right_dist): #MOVE RIGHT
			forwardAngle += 90
			moveTurn(90, 50, [leftMotor, rightMotor])
			stepForward(50, _DISTANCE_BETWEEN_WALLS) #Move forward enough to prevent turning on the spot.
		elif hasFront(front_dist) and hasRight(right_dist): #Choice of forward & right, will turn RIGHT
			forwardAngle += 90
			moveTurn(90, 50, [leftMotor, rightMotor])
			stepForward(50, _DISTANCE_BETWEEN_WALLS) #Move forward enough to prevent turning on the spot.
		elif not hasFront(front_dist) and not hasRight(right_dist): #Rotate 180 degrees and move forward again
			forwardAngle -= 180
			moveTurn(180, 50, [leftMotor, rightMotor])
		try:
			c = sys.stdin.read(1)
			"""print "Current char", repr(c)
			#The function tester!
			if(c == 'w'):
				moveFoward(50,50)
			elif(c == 'a'):
				moveTurn(-90, 50)
			elif(c == 'd'):
				moveTurn(90, 50)
			elif(c == 's'):
				moveFoward(-50,-50)
			elif(c == 'r'):
				stepForward(5,50)
			elif(c == 'f'):
				stepForward(-5,50)
			elif(c == 'c'):
				rightMotor.stop()
				leftMotor.stop()
				
		#printSensors()"""
			
		except IOError: pass
finally:
	termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
	fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

rightMotor.stop()
leftMotor.stop()
sensorMotor.stop()	