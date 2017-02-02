import time
import signal
import getch

pwmDev = "/dev/pi-blaster"
motorPin = 17
steeringPin = 27
gearPin = 22
MAX_SPEED = 100
MIN_SPEED = -100
MAX_STEERING = 125
MIN_STEERING = -25
ZERO_SPEED = 46
ZERO_STEERING = 50

class Truck:
	def __init__(self):
		self.Reset()

	def Reset(self):
		self.speed = ZERO_SPEED
		self.steering = ZERO_STEERING
		self.gear = 1

	def Update(self):
		SetServo(motorPin, self.speed)
		SetServo(steeringPin, self.steering)
		SetServo(gearPin, self.gear)

	def Shift(self, diff):
		gear = self.gear + diff
		if gear < 1:
			gear = 1
		elif gear > 3:
			gear = 3
		self.gear = gear

	def SetSpeed(self, speed):
		if speed > MAX_SPEED:
			speed = MAX_SPEED
		elif speed < MIN_SPEED:
			speed = MIN_SPEED
		self.speed = speed

	def SetSteering(self, steering):
		if steering > MAX_STEERING:
			steering = MAX_STEERING
		elif steering < MIN_STEERING:
			steering = MIN_STEERING
		self.steering = steering

def SetServo(pin, angle):
        duty = 0.15 + float(angle) / 100 * 0.05
	f = open(pwmDev, "w")
	command = "{}={}\n".format(pin, duty)
	f.write(command)
	f.close()

def ResetTruck():
	SetServo(motorPin, ZERO_SPEED)
	SetServo(steeringPin, ZERO_STEERING)
	SetGear(1)


def SetGear(gear):
	if gear == 3:
		gearSignal = 70
	elif gear == 2:
		gearSignal = 0
	elif gear == 1:
		gearSignal = -100
        else:
        	# Default to low gear
        	gearSignal = -100
	SetServo(gearPin, gearSignal)

def InterruptHandler(sig, frame):
	signal.signal(signal.SIGINT, signal.SIG_IGN)
	print("Interrupted, reset servo...")
	# Return to neutral
	ResetTruck()
	time.sleep(0.4)
	exit(0)

signal.signal(signal.SIGINT, InterruptHandler)

def KeyboardControl():
        truck = Truck()
	motorSpeed = ZERO_SPEED
	steeringAngle = ZERO_STEERING
	gear = 1
	truck.Reset()
	truck.Update()
	while "pigs can't fly":
		key = getch.getch()
		if key == '\x1b':
			key = getch.getch()
			if key == '[':
				key = getch.getch()
				if key == 'A':
					motorSpeed += 2
                                        truck.SetSpeed(motorSpeed)
				elif key == 'B':
					motorSpeed -= 2
					truck.SetSpeed(motorSpeed)
				elif key == 'C':
					steeringAngle += 1
					truck.SetSteering(steeringAngle)
				elif key == 'D':
					steeringAngle -= 1
					truck.SetSteering(steeringAngle)
				else:
					print("Unknown key {}".format(key))
		elif key == 'a':
                	if gear < 3:
				gear += 1
		elif key == 'z':
			if gear > 1:
				gear -= 1
		elif key == 'x':
                        motorSpeed = ZERO_SPEED
			steeringAngle = ZERO_STEERING
			gear = 1
			truck.Reset()
		elif key == '\x03':
			InterruptHandler(None, None)
			exit(0)
		else:
			print("Unknown key {}".format(key))
			motorSpeed = ZERO_SPEED
			steeringAngle = ZERO_STEERING
			gear = 1
			truck.Reset()
		print("Motor: {}, Steering: {}, Gear: {}".format(motorSpeed, steeringAngle, gear))
		truck.Update()

def Loop():
	print("Entering Loop function")
	delay = 0.1
	while "pigs can't fly":
		for angle in range(-10, 120, 1):
			SetServo(servoPin, angle)
			time.sleep(delay)
		for angle in range(120, -10, -1):
			SetServo(servoPin, angle)
			time.sleep(delay)

#KeyboardControl()
