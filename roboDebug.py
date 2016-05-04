#!/usr/bin/python

#Possible loops.
#Possible 4 way intersections
#Probably right angled walls.

_DISTANCE_BETWEEN_WALLS = 42 #cm
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

# Connect touch sensors.
#us = UltrasonicSensor(); assert us.connected
#ls = ColorSensor(); assert ls.connected
gs = GyroSensor(); assert gs.connected

gs.mode = 'GYRO-ANG'

# We will need to check EV3 buttons state.
btn = Button()

"""class TurnNode():
    def __init__(self):
		self.id = id(self)
	id
    distance1 = 0
    distance2 = 0
    angle1 = 0
    angle2 = 0
	
class PathNode(turnID):
	def __init__(self, turnID):
		self.turnID = turnID
	turnID
	startAngle = 0 #Angle the node is at with respect to start
	hasTraversed = False

"""
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
def forwardMonitor():
	pass

#Function to turn a specific angle
#Must have an angle in degrees, a valid gyro and speed
def moveTurn(angle, speed):
	currentAngle = gyroValue()
	print "current angle:", currentAngle
	finishedAngle = (currentAngle + angle + getTurnRecalibration())%360
	#turn until we get to the angle we want
	while(gyroValue() < finishedAngle - _ERROR_BOUNDS_GS or gyroValue() > finishedAngle + _ERROR_BOUNDS_GS):
		#TODO check logic
		print gyroValue(), " ", finishedAngle
		if(angle < 0):
			#TODO check direction
			rightMotor.run_direct(duty_cycle_sp=speed*1)
			leftMotor.run_direct(duty_cycle_sp=speed*-1)
		elif (angle > 0):
			#TODO check direction
			rightMotor.run_direct(duty_cycle_sp=speed*-1)
			leftMotor.run_direct(duty_cycle_sp=speed*1)
	print "finished angle", gyroValue()
	setTurnRecalibration(finishedAngle, gyroValue()) #Set the error in our turning.
	rightMotor.stop()
	leftMotor.stop()
			
			
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

try:
	print "ready!";
	while 1:
		try:
			c = sys.stdin.read(1)
			print "Current char", repr(c)
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
				
		#printSensors()
			
		except IOError: pass
finally:
	termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
	fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
