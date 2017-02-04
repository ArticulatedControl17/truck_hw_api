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

GEAR_1_SIGNAL = -100
GEAR_2_SIGNAL = 0
GEAR_3_SIGNAL = 70

class Truck:
	def __init__(self):
		self.reset()

	def reset(self):
		self.speed = ZERO_SPEED
		self.steering = ZERO_STEERING
		self.gear_signal = GEAR_1_SIGNAL

	def update(self):
		setServo(motorPin, self.speed)
		setServo(steeringPin, self.steering)
		setServo(gearPin, self.gear)

	def setGear(self, gear):
		if gear == 3:
			gear_signal = GEAR_3_SIGNAL
		elif gear == 2:
			gear_signal = GEAR_2_SIGNAL
		elif gear == 1:
			gear_signal = GEAR_1_SIGNAL
		else:
        	# Default to low gear
			gear_signal = GEAR_1_SIGNAL

	def setSpeed(self, speed):
		if speed > MAX_SPEED:
			speed = MAX_SPEED
		elif speed < MIN_SPEED:
			speed = MIN_SPEED
		self.speed = speed

	def setSteering(self, steering):
		if steering > MAX_STEERING:
			steering = MAX_STEERING
		elif steering < MIN_STEERING:
			steering = MIN_STEERING
		self.steering = steering


def setServo(pin, angle):
	duty = 0.15 + float(angle) / 100 * 0.05
	f = open(pwmDev, "w")
	command = "{}={}\n".format(pin, duty)
	f.write(command)
	f.close()


	
	
def interruptHandler(sig, frame):
	signal.signal(signal.SIGINT, signal.SIG_IGN)
	print("Interrupted, reset servo...")
	# Return to neutral
	truck = Truck()
	truck.resetTruck()
	time.sleep(0.4)
	exit(0)

signal.signal(signal.SIGINT, interruptHandler)

def keyboardControl():
	truck = Truck()
	motorSpeed = ZERO_SPEED
	steeringAngle = ZERO_STEERING
	gear = 1
	truck.reset()
	truck.update()
	while "pigs can't fly":
		key = getch.getch()
		if key == '\x1b':
			key = getch.getch()
			if key == '[':
				key = getch.getch()
				if key == 'A':
					motorSpeed += 2
				elif key == 'B':
					motorSpeed -= 2
				elif key == 'C':
					steeringAngle += 1
				elif key == 'D':
					steeringAngle -= 1
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
			truck.reset()
		elif key == '\x03':
			interruptHandler(None, None)
			truck.reset()
			exit(0)
		else:
			print("Unknown key {}".format(key))
			motorSpeed = ZERO_SPEED
			steeringAngle = ZERO_STEERING
			gear = 1
			truck.Reset()
		
		print("Motor: {}, Steering: {}, Gear: {}".format(motorSpeed, steeringAngle, gear))
		
		truck.setSpeed(motorSpeed)
		truck.setSteering(steeringAngle)
		truck.setGear(gear)
		truck.update()

