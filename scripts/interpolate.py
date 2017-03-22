from scipy.interpolate import interp1d
import numpy as np
import measurements as m
import rospy

generated = False

#to how many decimal points should points to generated
ANGLE_PRECISION = 1
F_SPEED_PRECISION = 2
B_SPEED_PRECISION = 2

min_angle = 0
max_angle = 0
min_speed = 0
max_speed = 0

max_trailersignal = 0
min_trailersignal = 0

#lookup-tables for translating angles/velocity to commands to the truck
angle_dict = {}
f_speed_dict = {}
b_speed_dict = {}

trailer_dict = {}

#reads the measure points in measurements.py and interpolates
def generateDictionaries():
    global generated
    global angle_dict, f_speed_dict, b_speed_dict,\
           max_angle, min_angle, max_speed, min_speed, trailer_dict, max_trailersignal, min_trailersignal
    if generated:
        return
    
    # ------------angle-----------

    #get interpolate function
    fA = interp1d(m.angleMeasurements, m.angleInputs)

    maxA = max(m.angleMeasurements)
    minA = min(m.angleMeasurements)

    #generate points
    xA = np.linspace(minA, maxA, 1+(maxA - minA) * (10 ** ANGLE_PRECISION)).tolist()
    yA = fA(xA)

    #round to remove floating point errors
    xA = [round(x, ANGLE_PRECISION) for x in xA]
    yA = [round(y, 2) for y in yA]

    max_angle = max(xA)
    min_angle = min(xA)

    angle_dict = dict(zip(xA, yA))


    # ----------------forward speed----------------
    #v = s/t
    speedsF = [0] + map (lambda x: m.forwardDistance/float(x), m.forwardMeasurements)
    inputsF = [m.zeroSpeedForward] + m.forwardInputs

    maxS = max(speedsF)


    fF = interp1d(speedsF, inputsF)

    xF = np.linspace(0, maxS, 1+(maxS * (10 ** F_SPEED_PRECISION))).tolist()
    yF = fF(xF)

    xF = [round(x, F_SPEED_PRECISION) for x in xF]
    yF = [round(y, 2) for y in yF]

    f_speed_dict = dict(zip(xF,yF))
    max_speed = max(xF)


    # ------------------backward speed-----------
    speedsB = [0] + map (lambda x: m.backwardDistance/float(x), m.backwardMeasurements)
    inputsB = [m.zeroSpeedBackward] + m.backwardInputs

    minS = max(speedsB)

    fB = interp1d(speedsB, inputsB)

    xB = np.linspace(0, minS, 1+(minS * (10 ** B_SPEED_PRECISION))).tolist()
    yB = fB(xB)

    xB = [round(x, B_SPEED_PRECISION) for x in xB]
    yB = [round(y,2) for y in yB]

    b_speed_dict = dict(zip(map(lambda x: -x, xB),yB))

    min_speed = -max(xB)

    generated = True
    
    # -----------------trailer sensor----------------------
    
    fT = interp1d(m.trailer_signals, m.trailer_angles)

    maxT = max(m.trailer_signals)
    minT = min(m.trailer_signals)

    #generate points
    xT = np.linspace(minT, maxT, 1+(maxT - minT)).tolist()
    yT = fT(xT)

    #round to remove floating point errors
    xT = [round(x) for x in xT]
    yT = [round(y, 2) for y in yT]

    max_trailersignal = max(xT)
    min_trailersignal = min(xT)

    trailer_dict = dict(zip(xT, yT))
    

def getSteeringCmd(phi):
    global generated
    if not generated:
        generateDictionaries()

    
    if phi > max_angle :
        phi = max_angle
    elif phi < min_angle:
        phi = min_angle

    return angle_dict[round(phi, ANGLE_PRECISION)]


def getTrailerAngle(signal):
    global generated
    if not generated:
        generateDictionaries()

    
    if signal > max_trailersignal :
        signal = max_trailersignal
    elif signal < min_trailersignal:
        signal = min_trailersignal

    return trailer_dict[round(signal)]

def getSpeedCmd(v):
    if not generated:
        generateDictionaries()

    if v > max_speed:
        v = max_speed
    elif v < min_speed:
        v = min_speed


    if v >= 0:
        return f_speed_dict[round(v, F_SPEED_PRECISION)]
    else:
        return b_speed_dict[round(v, B_SPEED_PRECISION)]



def setRosParams():
    #set rospy global parameters
    rospy.set_param('min_angle', min_angle)
    rospy.set_param('max_angle', max_angle)
    rospy.set_param('min_speed', min_speed)
    rospy.set_param('max_speed', max_speed)
