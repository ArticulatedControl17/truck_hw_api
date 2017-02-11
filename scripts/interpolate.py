from scipy.interpolate import interp1d
import numpy as np
import measurements as m
import rospy

generated = False

ANGLE_PRECISION = 1
F_SPEED_PRECISION = 2
B_SPEED_PRECISION = 2

min_angle = 0
max_angle = 0
min_speed = 0
max_speed = 0

angle_dict = {}
f_speed_dict = {}
b_speed_dict = {}


def generateDictionaries():

    if generated:
        return
    
    #angle
    fA = interp1d(m.angleMeasurements, m.angleInputs)

    maxA = max(m.angleMeasurements)
    minA = min(m.angleMeasurements)

    xA = np.linspace(minA, maxA, (maxA - minA) * (10 ** ANGLE_PRECISION)).tolist()
    yA = fA(xA)

    xA = [round(x, ANGLE_PRECISION) for x in xA]
    yA = [round(y, 2) for y in yA]

    max_angle = max(xA)
    min_angle = min(xA)

    angle_dict = dict(zip(xA, yA))


    #forward speed
    speedsF = [0] + map (lambda x: m.forwardDistance/float(x), m.forwardMeasurements)
    inputsF = [m.zeroSpeedForward] + m.forwardInputs

    maxS = max(speedsF)


    fF = interp1d(speedsF, inputsF)

    xF = np.linspace(0, maxS, (maxS * (10 ** F_SPEED_PRECISION))).tolist()
    yF = fF(xF)

    xF = [round(x, F_SPEED_PRECISION) for x in xF]
    yF = [round(y, 2) for y in yF]

    f_speed_dict = dict(zip(xF,yF))
    max_speed = max(xF)


    #backward speed
    speedsB = [0] + map (lambda x: m.backwardDistance/float(x), m.backwardMeasurements)
    inputsB = [m.zeroSpeedBackward] + m.backwardInputs

    minS = max(speedsB)

    fB = interp1d(speedsB, inputsB)

    xB = np.linspace(0, minS, (minS * (10 ** B_SPEED_PRECISION))).tolist()
    yB = fB(xB)

    xB = [round(x, B_SPEED_PRECISION) for x in xB]
    yB = [round(y,2) for y in yB]

    b_speed_dict = dict(zip(map(lambda x: -x, xB),yB))
    min_speed = -max(xB)

    generated = True


def getSteeringCmd(phi):
    if not generated:
        generateDictionaries()
    
    if phi > self.MAX_ANGLE :
        phi = self.MAX_ANGLE
    elif phi < self.MIN_ANGLE:
        phi = self.MIN_ANGLE


    return self.angle_dict[round(phi, ANGLE_PRECISION)]


def getSpeedCmd(v):
    if not generated:
        generateDictionaries()

    if v > self.MAX_SPEED:
        v = self.MAX_SPEED
    elif v < self.MIN_SPEED:
        v = self.MIN_SPEED


    if v >= 0:
        return f_speed_dict[round(v, F_SPEED_PRECISION)]
    else:
        return b_speed_dict[round(v, B_SPEED_PRECISION)]



def setRosParams():
    #set rospy global parameters
    rospy.set_param_raw('min_angle', min_angle)
    rospy.set_param_raw('max_angle', max_angle)
    rospy.set_param_raw('min_speed', min_speed)
    rospy.set_param_raw('max_speed', max_speed)