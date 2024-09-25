#!/usr/bin/env python3

from controller import Robot, Keyboard, Motion
import rospy
from std_msgs.msg import Float64, String

class Nao(Robot):
    PHALANX_MAX = 8
    
    def loadMotionFiles(self):
        self.handWave = Motion('../../motions/HandWave.motion')
        self.forwards = Motion('../../motions/Forwards50.motion')
        self.backwards = Motion('../../motions/Backwards.motion')
        self.sideStepLeft = Motion('../../motions/SideStepLeft.motion')
        self.sideStepRight = Motion('../../motions/SideStepRight.motion')
        self.turnLeft60 = Motion('../../motions/TurnLeft60.motion')
        self.turnRight60 = Motion('../../motions/TurnRight60.motion')

    def startMotion(self, motion):
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()
        motion.play()
        self.currentlyPlaying = motion

    def printAcceleration(self):
        acc = self.accelerometer.getValues()
        print('----------accelerometer----------')
        print('acceleration: [ x y z ] = [%f %f %f]' % (acc[0], acc[1], acc[2]))

    def printGyro(self):
        vel = self.gyro.getValues()
        print('----------gyro----------')
        print('angular velocity: [ x y ] = [%f %f]' % (vel[0], vel[1]))
        
    def printGps(self):
        p = self.gps.getValues()
        print('----------gps----------')
        print('position: [ x y z ] = [%f %f %f]' % (p[0], p[1], p[2]))

    def printInertialUnit(self):
        rpy = self.inertialUnit.getRollPitchYaw()
        print('----------inertial unit----------')
        print('roll/pitch/yaw: [%f %f %f]' % (rpy[0], rpy[1], rpy[2]))

    def printFootSensors(self):
        fsv = [self.fsr[0].getValues(), self.fsr[1].getValues()]
        l = []
        r = []
        for i in range(0, 4):
            l.append(fsv[0][i])
            r.append(fsv[1][i])
        print('----------foot sensors----------')
        print('left: %s' % str(l))
        print('right: %s' % str(r))

    def printFootBumpers(self):
        ll = self.lfootlbumper.getValue()
        lr = self.lfootrbumper.getValue()
        rl = self.rfootlbumper.getValue()
        rr = self.rfootrbumper.getValue()
        print('----------foot bumpers----------')
        print('left: [%d %d], right: [%d %d]' % (ll, lr, rl, rr))

    def printUltrasoundSensors(self):
        dist = [self.us[0].getValue(), self.us[1].getValue()]
        print('-----ultrasound sensors-----')
        print('left: %f m, right %f m' % (dist[0], dist[1]))

    def setAllLedsColor(self, rgb):
        for i in range(0, len(self.leds)):
            self.leds[i].set(rgb)
        
    def closeAndOpenHands(self, countSpikes):
        for i in range(countSpikes):
            self.setHandsAngle(0.96)
            rospy.sleep(1.5) 
            self.setHandsAngle(0.00)
            rospy.sleep(1.5)
            print('X%d' %(i))

    def setHandsAngle(self, angle):
        for i in range(0, self.PHALANX_MAX):
            clampedAngle = max(min(angle, self.maxPhalanxMotorPosition[i]), self.minPhalanxMotorPosition[i])
            if len(self.rphalanx) > i and self.rphalanx[i] is not None:
                self.rphalanx[i].setPosition(clampedAngle)
            if len(self.lphalanx) > i and self.lphalanx[i] is not None:
                self.lphalanx[i].setPosition(clampedAngle)

    def findAndEnableDevices(self):
        self.timeStep = int(self.getBasicTimeStep())

        self.cameraTop = self.getDevice("CameraTop")
        self.cameraBottom = self.getDevice("CameraBottom")
        self.cameraTop.enable(4 * self.timeStep)
        self.cameraBottom.enable(4 * self.timeStep)

        self.accelerometer = self.getDevice('accelerometer')
        self.accelerometer.enable(4 * self.timeStep)

        self.gyro = self.getDevice('gyro')
        self.gyro.enable(4 * self.timeStep)

        self.gps = self.getDevice('gps')
        self.gps.enable(4 * self.timeStep)

        self.inertialUnit = self.getDevice('inertial unit')
        self.inertialUnit.enable(self.timeStep)

        self.us = [self.getDevice('Sonar/Left'), self.getDevice('Sonar/Right')]
        for i in range(2):
            self.us[i].enable(self.timeStep)

        self.fsr = [self.getDevice('LFsr'), self.getDevice('RFsr')]
        for i in range(2):
            self.fsr[i].enable(self.timeStep)

        self.lfootlbumper = self.getDevice('LFoot/Bumper/Left')
        self.lfootrbumper = self.getDevice('LFoot/Bumper/Right')
        self.rfootlbumper = self.getDevice('RFoot/Bumper/Left')
        self.rfootrbumper = self.getDevice('RFoot/Bumper/Right')
        self.lfootlbumper.enable(self.timeStep)
        self.lfootrbumper.enable(self.timeStep)
        self.rfootlbumper.enable(self.timeStep)
        self.rfootrbumper.enable(self.timeStep)

        self.leds = [
            self.getDevice('ChestBoard/Led'),
            self.getDevice('RFoot/Led'),
            self.getDevice('LFoot/Led'),
            self.getDevice('Face/Led/Right'),
            self.getDevice('Face/Led/Left'),
            self.getDevice('Ears/Led/Right'),
            self.getDevice('Ears/Led/Left')
        ]

        self.lphalanx = []
        self.rphalanx = []
        self.maxPhalanxMotorPosition = []
        self.minPhalanxMotorPosition = []
        for i in range(0, self.PHALANX_MAX):
            self.lphalanx.append(self.getDevice("LPhalanx%d" % (i + 1)))
            self.rphalanx.append(self.getDevice("RPhalanx%d" % (i + 1)))

            self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
            self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())

        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)

    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = False
        self.findAndEnableDevices()
        self.loadMotionFiles()
        
        rospy.init_node('controller', anonymous=True)
        self.pub = rospy.Publisher('motor', Float64, queue_size=10)
        rospy.Subscriber("spikes", String, self.callback) 
        
    def callback(self, data):
        print("I heard %s", data.data)
        count = int(data.data)
        self.closeAndOpenHands(count)
        
    def run(self):
        rate = rospy.Rate(0.1) 
        while not rospy.is_shutdown():
            rospy.sleep(0.1)   
            if self.step(self.timeStep) == -1:
                break

robot = Nao()
robot.run()
