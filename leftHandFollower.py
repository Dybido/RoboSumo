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
	print "currentDist ", currentDist
	#move until we get to the distance we want
	while(us.value() < finishedDist - _ERROR_BOUNDS_US or us.value() > finishedDist + _ERROR_BOUNDS_GS):
		speedlft = speed
		speedrgt = speed
		print "getto ", us.value(), finishedDist
		"""if(gyroValue() < forwardAngle):
			speedrgt = (speed/5)*4
		elif(gyroValue() > forwardAngle):
			speedlft = (speed/5)*4"""
		if(us.value() < finishedDist):
			#We want to move back because we've overshot
			moveFoward(-speedrgt,-speedlft)
		elif(us.value() > finishedDist):
			#move forward until we're in bounds
			moveFoward(speedlft,speedrgt)
	leftMotor.stop()
	rightMotor.stop()

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
	finishedAngle = (currentAngle + angle + getTurnRecalibration())%360
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
	setTurnRecalibration(finishedAngle, gyroValue()) #Set the error in our turning.
	for motor in motorList:
		motor.stop()
	

def hasLeft(gs_val):
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

def detectLeft():
	"""
	Returns the distance value of the right 
	"""
	sensorMotor.run_to_abs_pos(position_sp=-90, stop_command="brake")
	while any(m.state for m in ([sensorMotor])):
		sleep(0.1)
	sleep(0.2)
	left_dist = us.value()
	return left_dist

def detectFront():
	"""
	Returns the distance value of the front
	"""
	sensorMotor.run_to_abs_pos(position_sp=0, stop_command="brake")
	while any(m.state for m in ([sensorMotor])):
		sleep(0.1)
	sleep(0.2)
	front_dist = us.value()
	return front_dist

def detectSkew(currAngle):
	if currAngle < forwardAngle :
		diff = (forwardAngle - currAngle) % 360
		while diff != 0:
			moveFoward(55, 47)
			diff = (forwardAngle - currAngle) % 360
	else: #currAngle > forwardAngle
		diff = (currAngle - forwardAngle) % 360
		while diff != 0:
			moveFoward(55, 47)
			diff = (currAngle - forwardAngle) % 360
		
			
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
		FRONT| RIGHT| MOVE
		---------------------
		TRUE   FALSE  FORWARD
		FALSE  TRUE   TURN RIGHT
		TRUE   TRUE   TURN RIGHT 
		FALSE  FALSE  TURN RIGHT
		"""
		left_dist = detectLeft()
		front_dist = detectFront()
		print "right ", left_dist
		print "front ", front_dist

		if hasFront(front_dist) and not hasLeft(left_dist): #MOVE FORWARD
			#print front_dist
			#while detectFront() >= MIN_DIST or not hasRight(left_dist):
			#forwardMonitor(forwardAngle%360, 50)
			
			moveFoward(47,47)
			"""currentDir = gyroValue()
			if(forwardAngle + 5 > 360 || forwardAngle - 5 < 0):
				#shift angles to work at a better range
				changedForwardAngle = (forwardAngle-180)%360
				changedCurrentDir = (currentDir-180)%360
				
				if((changedCurrentDir - 5) > changedForwardAngle):
					# print('right')
					rightMotor.duty_cycle_sp = 40
				elif((changedCurrentDir + 5) < changedForwardAngle):
					leftMotor.duty_cycle_sp = 40
			else:
				if((currentDir - 5) > forwardAngle):
					# print('right')
					rightMotor.duty_cycle_sp = 40
				elif((currentDir + 5) < forwardAngle):
					leftMotor.duty_cycle_sp = 40"""
			
			
			#direction = gyroValue();
			#print "gyrovalue: ", currentDir, forwardAngle
			
		elif not hasFront(front_dist) and hasLeft(left_dist): #MOVE LEFT
			print "turns right 1"
			forwardAngle-=90
			#moveTurn(90, 50, [leftMotor, rightMotor])
			leftMotor.run_to_rel_pos(position_sp=225, stop_command="brake")
			rightMotor.run_to_rel_pos(position_sp=-225, stop_command="brake")
			while any(m.state for m in (leftMotor, rightMotor)):
				sleep(0.1)
			moveFoward(47,47)
			currAngle = gyroValue()
			detectSkew(currAngle)
			sleep(2)
			#stepForward(_DISTANCE_BETWEEN_WALLS, 50) #Move forward enough to prevent turning on the spot.
		elif hasFront(front_dist) and hasLeft(left_dist): #Choice of forward & right, will turn RIGHT
			print "turns right 2"
			forwardAngle-=90
			#moveTurn(90, 50, [leftMotor, rightMotor])
			leftMotor.run_to_rel_pos(position_sp=225, stop_command="brake")
			rightMotor.run_to_rel_pos(position_sp=-225, stop_command="brake")
			while any(m.state for m in (leftMotor, rightMotor)):
				sleep(0.1)
			moveFoward(47,47)
			currAngle = gyroValue()
			detectSkew(currAngle)
			sleep(2)
			#stepForward(_DISTANCE_BETWEEN_WALLS, 50) #Move forward enough to prevent turning on the spot.
		elif not hasFront(front_dist) and not hasLeft(left_dist): #Rotate 180 degrees and move forward again
			forwardAngle+=180
			leftMotor.run_to_rel_pos(position_sp=-480, stop_command="brake")
			rightMotor.run_to_rel_pos(position_sp=480, stop_command="brake")
			while any(m.state for m in (leftMotor, rightMotor)):
				sleep(0.1)
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
			"""elif(c == 'a'):
				moveTurn(-90, 50, [leftMotor, rightMotor])
			elif(c == 'd'):
				moveTurn(90, 50, [leftMotor, rightMotor])
			elif(c == 's'):
				moveFoward(-50,-50)
			elif(c == 'r'):
				stepForward(_DISTANCE_BETWEEN_WALLS, 50)
			elif(c == 'f'):
				stepForward(-_DISTANCE_BETWEEN_WALLS, 50)
			elif(c == 'i'):
				temp = detectFront()
				print temp
				print hasFront(temp)
			elif(c == 'l'):
				temp = detectRight()
				print temp
				print hasRight(temp)
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