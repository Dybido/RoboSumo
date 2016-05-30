#!/usr/bin/python

_DISTANCE_BETWEEN_WALLS = 300 #mm
_ERROR_BOUNDS_GS = 1 #degrees
_ERROR_DRIFT_LEFT_GS = 0 #degrees per rotation
_ERROR_DRIFT_RIGHT_GS = -1 #degrees per rotation
_SENSOR_MOTOR_FORWARD = 0 #use device browser to calibrate
_STANDARD_SPEED = 60 #duty cycle right, left motors
_SENSOR_SPEED = 50 #duty cycle sensor motor
_CLAMP_SPEED = 90 #duty cycle clamp motor
_POSITIONS_PER_BLOCK = 875 #position_sp per 42cm block

from time   import sleep
import termios, fcntl, sys, os
fd = sys.stdin.fileno()

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from ev3dev.auto import *

#Connect motors
rightMotor = LargeMotor(OUTPUT_A)
leftMotor = LargeMotor(OUTPUT_B)
sensorMotor = MediumMotor(OUTPUT_C) #SENSOR
clampMotor = LargeMotor(OUTPUT_D) #Clamp

#Connect sensors
us = UltrasonicSensor(); assert us.connected
ls = ColorSensor(); assert ls.connected
gs = GyroSensor(); assert gs.connected

#print "Start calibrate"
#gs.mode = 'GYRO-CAL'
#sleep(5)
#print "End calibrate"


#Set sensor modes
gs.mode = 'GYRO-ANG'
ls.mode = 'COL-COLOR'

# We will need to check EV3 buttons state.
btn = Button()

#intersection related objects
nodeID = 0
nodeList = []
class PathNode():
	#A path node is a node which defines a
	#path when a two way or more intersection is
	#reached. It stores an ID, a parentID, angle
	#of traversal relative to parent and whether it
	#has been visited or not.
	def __init__(self, parentID, dir):
		global nodeID
		self.ID = nodeID
		nodeID += 1
		self.parentID = parentID
		self.directionAngle = dir
	ID = -1
	parentID = -1
	directionAngle = 0
	visited = False


#Begin traversing a path
def startPath(path, offsetAng):
	#offsetAng allows us to turn from different starting points
	turnCorner(path.directionAngle+offsetAng)

#Any turning now goes through this...
def turnCorner(inAngle):
	print "Turning ", inAngle
	rotateRobot(inAngle, _STANDARD_SPEED)
	moveForward(_STANDARD_SPEED, _STANDARD_SPEED)
	
#Simply to mod gyro value
def gyroValue():
	return gs.value()%360

#Function to move forward continuously
#Requires speed for both wheels
def moveForward(lft, rgt):
	#rightMotor.run_direct(duty_cycle_sp=rgt)
	#leftMotor.run_direct(duty_cycle_sp=lft)
	finalPosLeft = leftMotor.position+_POSITIONS_PER_BLOCK
	finalPosRight = rightMotor.position+_POSITIONS_PER_BLOCK

	leftMotor.run_to_abs_pos(position_sp=finalPosLeft, stop_command="brake")
	rightMotor.run_to_abs_pos(position_sp=finalPosRight, stop_command="brake")
	while any(m.state for m in ([rightMotor, leftMotor])):
		#as most time is spent in here, also run direction correction code
		print "RUNNING"
		detectSkew()
		leftMotor.run_to_abs_pos(position_sp=finalPosLeft, stop_command="brake")
		rightMotor.run_to_abs_pos(position_sp=finalPosRight, stop_command="brake")
		
		if(rightMotor.state == False):
			print "right"
			leftMotor.stop()
			rightMotor.stop()
			break
		elif(leftMotor.state == False):
			print "left"
			leftMotor.stop()
			rightMotor.stop()
			break
		if(abs(leftMotor.position-finalPosLeft) < 5 or abs(rightMotor.position-finalPosRight) < 5):
			print "close and crappy"
			leftMotor.stop()
			rightMotor.stop()
			break
		
	rightMotor.duty_cycle_sp = _STANDARD_SPEED
	leftMotor.duty_cycle_sp = _STANDARD_SPEED
	#rightMotor.stop()
	#leftMotor.stop()
	

#Function to turn based on globalAngle
#needs a valid gyro, left and right motors and speed
def rotateRobot(inAngle, speed):
	"""Still currently rotates the incorrect way"""
	global forwardAngle
	forwardAngle += inAngle
	
	rotateAmount = 205/90
	rotateError = _ERROR_BOUNDS_GS #degrees
	currentGyro = gyroValue()
	
	angle = (forwardAngle-currentGyro)%360
	#print "Start: ", forwardAngle, currentGyro, angle

	leftMotor.duty_cycle_sp = speed
	rightMotor.duty_cycle_sp = speed
	
	print "Turning!!!"
	if(angle > 180):
		#left
		angle = -(360-angle)
		forwardAngle -= _ERROR_DRIFT_LEFT_GS
	else:
		forwardAngle += _ERROR_DRIFT_RIGHT_GS
		
	#print "newRotate", angle
	
	leftMotor.run_to_rel_pos(position_sp=angle*rotateAmount, stop_command="brake")
	rightMotor.run_to_rel_pos(position_sp=-angle*rotateAmount, stop_command="brake")
	while any(m.state for m in (leftMotor, rightMotor)):
		sleep(0.1)
		
	angleWant = (currentGyro+(angle)%360)%360
	angleHave = gyroValue()
	while((angleWant < angleHave - rotateError) or (angleWant > angleHave + rotateError)):
		angleDifference = angleWant-angleHave
		#print " "
		#print "AngleWant: ", angleWant
		#print "AngleHave: ", angleHave
		leftMotor.duty_cycle_sp = speed/2
		rightMotor.duty_cycle_sp = speed/2
		leftMotor.run_to_rel_pos(position_sp=angleDifference*rotateAmount, stop_command="brake")
		rightMotor.run_to_rel_pos(position_sp=-angleDifference*rotateAmount, stop_command="brake")
		angleHave = gyroValue()
	
	print "Accuracy: ", currentGyro+angle, gyroValue()
	leftMotor.duty_cycle_sp = speed
	rightMotor.duty_cycle_sp = speed
	

#Sensor related objects which allow to us to discover the walls around us
hasLeft = False
hasFront = True
hasRight = False
def sensorThread():
	"""Checks values on each side for intersections"""
	global hasLeft
	global hasFront
	global hasRight
	#TODO: sensor class to store values? or join
	hasLeft = hasNoWall(detectDistance(_SENSOR_MOTOR_FORWARD+90))
	hasFront = hasNoWall(detectDistance(_SENSOR_MOTOR_FORWARD))
	hasRight = hasNoWall(detectDistance(_SENSOR_MOTOR_FORWARD-90))
	#reset = detectDistance(_SENSOR_MOTOR_FORWARD+90)
	

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
	sleep(0.5)
	left_dist = us.value()
	return left_dist
	
def detectSkew():
	forwardAng = forwardAngle%360
	currAngle = gyroValue()

	diff = currAngle - forwardAng
	if(forwardAng < 45 or forwardAng > 315):
		diff = (currAngle + 180)%360 - (forwardAng + 180)%360 #dodgy as
	
	#print diff
	if(diff >= _ERROR_BOUNDS_GS):
		#print "right- needs left"
		leftMotor.duty_cycle_sp = _STANDARD_SPEED/2
	if(diff <= -_ERROR_BOUNDS_GS):
		#print "left - needs right"
		rightMotor.duty_cycle_sp = _STANDARD_SPEED/2
	else:
		leftMotor.duty_cycle_sp = _STANDARD_SPEED
		rightMotor.duty_cycle_sp = _STANDARD_SPEED
			
#Make sure that it's color_colour mode
#Robot has to be objectively close enough to clamp 
def detectCan():
	global goToStart
	global backToLast
	#Detects can and then uses clampMotor to grab the can
	if ls.value() == 5: 
		#clampMotor.run_direct(duty_cycle_sp=30, time_sp=500)
		print "CAN DETECTED!!!"
		#moveForward(30,30)
		leftMotor.run_to_rel_pos(position_sp=75, stop_command="brake")
		rightMotor.run_to_rel_pos(position_sp=75, stop_command="brake")
		sleep(1)
		
		clampMotor.run_to_abs_pos(position_sp=175, stop_command="brake")
		sleep(1)
		
		leftMotor.run_to_rel_pos(position_sp=-75, stop_command="brake")
		rightMotor.run_to_rel_pos(position_sp=-75, stop_command="brake")
		sleep(1)
		
		goToStart = True
		backToLast = True
		#turnCorner(180) #Careful! I removed this and haven't tested without!
	else:
		#print "No CAN detected this time!"
		pass
	
def victimScan():
	print "Start victim scan"
	#speed = leftMotor.duty_cycle_sp
	leftMotor.duty_cycle_sp = _STANDARD_SPEED
	rightMotor.duty_cycle_sp = _STANDARD_SPEED
	rotateRobot(-45, 50)
	detectCan()
	rotateRobot(30, 50)
	detectCan()
	rotateRobot(30, 50)
	detectCan()
	rotateRobot(30, 50)
	detectCan()
	rotateRobot(-45, 50)
	print "End victim scan"

#Print sensors		
def printSensors():
	print "Gyro ", gs.value()
	print "Ultra ", us.value()
	
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)

oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

sensorMotor.duty_cycle_sp = _SENSOR_SPEED
rightMotor.duty_cycle_sp = _STANDARD_SPEED
leftMotor.duty_cycle_sp = _STANDARD_SPEED
clampMotor.duty_cycle_sp = _CLAMP_SPEED
#start calibration mode to sit and recal gyro()

#Create our initial path
currentPath = PathNode(-1, 0)
nodeList.append(currentPath)

forwardAngle = gyroValue() #allow us to monitor the path as we are moving forward
backToLast = False
goToStart = False

#Allow us to toggle which directions we should choose for 'debugging'
leftToggle = True
frontToggle = True
rightToggle = True

print "starter: ", forwardAngle
try:
	while not btn.any():
		"""
		Run a thread for detecting left front and right.
		Join thread to be used by main thread which controls direction and motors
		
		"""
		
		"""
		Two different threads for sensing
		(Threads are weird things in python, dont attempt until rest is done)
		One thread has colour
		One thread has left/front/right sensing
		
		Main thread continually checks colour but waits
		for left/front/right to do the stuff below
		
		"""
		#Run detecting functions
		if(not goToStart):
			detectCan()
			#victimScan()
		sensorThread();
		print "Detection", hasLeft, hasFront, hasRight
		
		#Get values out of sensorThread specifying if theres walls or not
		#If this is true then there's an intersection at our location
		if(((hasLeft or hasRight) and hasFront) or (hasLeft and hasRight)):
			if(backToLast):
				print "Choosing next node from already created list"
				#backToLast is set when we're going back to a parent node
				#ie. dead-end or we've found the victim
				#The currentPath is still the node we last traversed 
				
				angleOffset = 180 #as we are coming from another node
				angleOffset -= currentPath.directionAngle #dir relative to parent 
				
				#Get the next non-transversed node and check its parent is same as currentPath's
				nextIndex = -1
				for i, j in enumerate(nodeList):
					if j.parentID == currentPath.parentID and not j.visited:
						nextIndex = i
				
				if(nextIndex != -1 and not goToStart):
					#goToStart is set when a victim has been found
					#We found a node on same parent to traverse!
					print "Chose current node:", nodeList[nextIndex].ID, ", parent: ", currentPath.parentID
					currentPath = nodeList[nextIndex]
					print "Checking parents are the same:", currentPath.parentID
					startPath(currentPath, angleOffset)
					backToLast = False
				else:
					#Our node doesn't have any more directions to go
					#Need to go back again so dont set backToLast to false
					#Turn back through to parents node by simply turning -directionAngle
					print "No more current nodes, going back to next parent"
					startPath(currentPath, 2*currentPath.directionAngle)
					#find parents index to set as current
					parentIndex = 0
					#Search for parents index
					for i, j in enumerate(nodeList):
						if j.ID == currentPath.parentID:
							parentIndex = i
								
					if(parentIndex == 0):
						#There's no more parents, we've either stuffed up or we've
						#finished the maze!
						print "FINISHED MAZE???"
						clampMotor.run_to_abs_pos(position_sp=0, stop_command="brake")
						Sound.tone([(1000, 500, 500)] * 1)
					
					currentPath = nodeList[parentIndex]
			else:
				#There a new intersection, we need to create a correct amount of nodes
				
				#TODO:
				#A better loop detection system :)
				
				currentPath.visited = True;
				#current index
				currentIndex = 0
				#work left to right so ensure we turn right to left
				if(hasLeft and leftToggle):
					newPath = PathNode(currentPath.ID, -90)
					print "Creating left node"
					#insert the node after our current parent
					for i, j in enumerate(nodeList):
						if j.ID == newPath.parentID:
							nodeList.insert(i+1,newPath)
							currentIndex = i
				if(hasFront and frontToggle):
					newPath = PathNode(currentPath.ID, 0)
					print "Creating front node"
					#insert the node after our current parent
					for i, j in enumerate(nodeList):
						if j.ID == newPath.parentID:
							nodeList.insert(i+1,newPath)
							currentIndex = i
				if(hasRight and rightToggle):
					newPath = PathNode(currentPath.ID, 90)
					#print currentPath.ID, " ", newPath.parentID, " ", newPath.ID
					print "Creating right node"
					#insert the node after our current parent
					for i, j in enumerate(nodeList):
						if j.ID == newPath.parentID:
							nodeList.insert(i+1,newPath)
							currentIndex = i
				
				#Inside the node we store our travel angle and other data...
				#Our decision is always to take the right-most path
				currentPath = nodeList[currentIndex+1]
				print "New current node:", currentPath.ID, ", parent: ", nodeList[currentIndex].ID
				print "Node list length: ", len(nodeList)
				
				#begin traversing the new right-most path
				startPath(currentPath, 0)
				
		elif(hasLeft or hasRight):
			#We need to turn the robot but no need to create a path/intersection
			#as there's no multi-way intersection.
			
			#victimScan()
			if(hasRight):
				print "Turning right!"
				turnCorner(90)
			else:
				print "Turning left!"
				turnCorner(-90)
		elif(not hasLeft and not hasFront and not hasRight):
			print "We are at a dead end, turn back!"
			#Need to go back to the last node and check the next value...
			#Go back to parent and do the second/third nodes.
			#If they don't exist (next node isnt sequential ID number) then go
			#back to the again parent node etc... while/for loops
			
			currentPath.visited = True
			#Scan for victim quickly
			victimScan()
			
			#Turn around
			turnCorner(180)
			
			#Set variable to go back to parent node
			backToLast = True
		else:
			#We're on the right track, just keep going forward until we reach an intersection
			#moveForward(50,50)
			turnCorner(0) #Attempt to keep robot going straight"""
		
		#Keyboard - used for testing components
		try:
			c = sys.stdin.read(1)
			print "Current char", repr(c)
			#The function tester/debugging
			if(c == 'c'):
				rightMotor.stop()
				leftMotor.stop()
				sensorMotor.stop()
				clampMotor.stop()
				#break
			elif(c == 'g'):
				rotateRobot(-90, _STANDARD_SPEED)
			elif(c == 'j'):
				rotateRobot(90, _STANDARD_SPEED)
			elif(c == 'h'):
				rotateRobot(180, _STANDARD_SPEED)
			elif(c == 'y'):
				moveForward(_STANDARD_SPEED,_STANDARD_SPEED)
			elif(c == 'q'):
				victimScan()
			elif(c == 'e'):
				sensorThread()
				print hasLeft, hasFront, hasRight
			elif(c == 'z'):
				clampMotor.run_to_rel_pos(position_sp=-130, stop_command="brake")
			elif(c == 'x'):		
				clampMotor.run_to_rel_pos(position_sp=130, stop_command="brake")
				
			elif(c == 'a'):
				leftToggle = not leftToggle
				print "left toggled:", leftToggle
			elif(c == 'w'):
				frontToggle = not frontToggle
				print "front toggled:", frontToggle
			elif(c == 'd'):
				rightToggle = not rightToggle
				print "eight toggled:", rightToggle
		#printSensors()"""
			
		except IOError: pass
finally:
	termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
	fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

rightMotor.stop()
leftMotor.stop()
sensorMotor.stop()