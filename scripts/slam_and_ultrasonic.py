#!usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
import math
import random
import numpy as np
class slam_sonar_lib:
    def __init__(self):
        self.radius_wheel = 0.0425
        self.b = 0.1974
        self.circumference = self.radius_wheel * 2 * math.pi
    def computeDifferent(self, angleStart, angleEnd):
        result = 0
        orientation = 0
        if angleStart >= 0 and angleEnd >=0:
            result = abs(angleStart - angleEnd)
            if angleStart > angleEnd:
                orientation = 0
            else:
                orientation = 1
        elif angleStart <= 0 and angleEnd <= 0:
            result = abs(angleStart - angleEnd)
            if angleStart > angleEnd:
                orientation = 0
            else:
                orientation = 1
        elif angleStart >= 0 and angleEnd <=0:
            result1 = 180 - angleStart + 180 + angleEnd
            result2 =  angleStart - angleEnd
            if result1 < result2:
                result = result1
                orientation = 1
            else:
                result = result2
                orientation = 0
        elif angleStart <=0 and angleEnd >= 0:
            result1 = 360 - angleEnd + angleStart
            result2 = angleEnd - angleStart
            if result1 < result2:
                result = result1
                orientation = 0
            else:
                result = result2
                orientation = 1
        return result, orientation
    def computeNewPose(self, prev_pose, deltaT, wLeft, wRight):
        # Theta: degree
        r = 0.0425
        b = 0.1974
        x = prev_pose[0] # m
        y = prev_pose[1] # m
        theta = prev_pose[2] * math.pi / 180 # degree -> rad
        velocity = r/2*wLeft + r/2*wRight # m/s
        omega = r/b * wLeft - r/b *wRight # rad/s

        x_new = x + velocity*math.cos(theta)*deltaT # compute x new
        y_new = y + velocity*math.sin(theta)*deltaT # compute y new
        theta_new = theta + omega*deltaT # compute theta new 
        theta_new = theta_new * 180 / math.pi # rad -> degree
        if theta_new > 180:
            theta_new = theta_new - 360 # degree
        if theta_new < -180:
            theta_new = theta_new + 360 # degree
        return [x_new, y_new, theta_new]
    def computeTarget(self, angleStart, difference, orientation):
        if orientation == 0:
            result = angleStart - difference
        else:
            result = angleStart + difference
        if result > 180:
            result = result - 360
        if result < -180:
            result = result + 360
        return result
    def extractPoint(self, x, y, orientation, distanceList):
        """
            Extract point from position, orientation, data sensor of robot
        """
        dataPoint = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
        theta = orientation
        if distanceList[0] != 2.5:
            distanceList[0] = distanceList[0] + 0.067
        if distanceList[1] != 2.5:
            distanceList[1] = distanceList[1] + 0.082
        if distanceList[2] != 2.5:
            distanceList[2] = distanceList[2] + 0.067
        if distanceList[3] != 2.5:
            distanceList[3] =  distanceList[3] + 0.082
        if 0 <= orientation <= 90:
            AO = distanceList[1]
            BO = distanceList[2]
            CO = distanceList[3]
            DO = distanceList[0]
        elif 90 < orientation <= 180:
            theta = orientation - 90
            AO = distanceList[0]
            BO = distanceList[1]
            CO = distanceList[2]
            DO = distanceList[3]
        elif -90 <= orientation < 0:
            theta = orientation + 90
            AO = distanceList[2]
            BO = distanceList[3]
            CO = distanceList[0]
            DO = distanceList[1]
        else:
            theta = orientation + 180
            AO = distanceList[3]
            BO = distanceList[0]
            CO = distanceList[1]
            DO = distanceList[2]
        theta = theta * math.pi/180
        if AO < 2.5:
            dataPoint[0][0] = x + AO * math.cos(theta)
            dataPoint[1][0] = y + AO * math.sin(theta)
        else:
            dataPoint[0][0] = 0
            dataPoint[1][0] = 0
        if BO < 2.5:
            dataPoint[0][1] = x - BO * math.sin(theta)
            dataPoint[1][1] = y + BO * math.cos(theta)
        else:
            dataPoint[0][1] = 0
            dataPoint[1][1] = 0
        if CO != 0:
            dataPoint[0][2] = x - CO * math.cos(theta)
            dataPoint[1][2] = y - CO * math.sin(theta)
        else:
            dataPoint[0][2] = 0
            dataPoint[1][2] = 0
        if DO != 0:
            dataPoint[0][3] = x + DO * math.sin(theta)
            dataPoint[1][3] = y - DO * math.cos(theta)
        else:
            dataPoint[0][3] = 0
            dataPoint[1][3] = 0

        return dataPoint
    def generateAngleList(self, angle):
        # generate four angle form initial angle
        angleList =  [angle -90 , angle , angle + 90, angle + 180]
        for i in range(len(angleList)):
            if angleList[i] > 180:
                angleList[i] = angleList[i] - 360
            elif angleList[i] < -180:
                angleList[i] = angleList[i] + 360
        return angleList
    def pidResponse(self, error, kp, ki, kd, sumError, prevError, deltaT):
        P = kp*error
        sumError = sumError + error * deltaT
        I = ki * sumError
        D = kd * (error - prevError) / deltaT
        response = P + I + D
        response = response
        sum = sumError
        prev = error 

        return response, sum, prev
    def addDataPoint(self,dataPointAll, dataPoint):
        for i in range(len(dataPoint[0])):
            dataPointAll[0].append(dataPoint[0][i])
            dataPointAll[1].append(dataPoint[1][i])
        
        return dataPointAll
    def addArray(self, initData, extraData):
        for i in range(len(extraData)):
            initData.append(extraData[i])

        return initData
    def computeSteps(self, deltaAngle):
        arc_length = self.b * (deltaAngle * math.pi / 180)
        steps = int(arc_length * 200 / self.circumference) + 1
        return steps
    def computeAngularVelocity(self, steps, orientation, stepsOfRevolution, deltaT):
        if orientation == 0: # turn right
            vLeft =  steps/deltaT # steps/s
            omegaLeft = 2*math.pi * vLeft / stepsOfRevolution # rad/s
            omegaRight = 0 # rad/s
        elif orientation == 1: # turn left
            vRight = steps/deltaT # steps/s
            omegaLeft = 0 # rad/s
            omegaRight = 2 * math.pi * vRight / stepsOfRevolution # rad/s
        elif orientation == -1 :# forward
            vMotor = steps/deltaT # steps/s
            omegaLeft = 2 * math.pi * vMotor / stepsOfRevolution # rad/s
            omegaRight = 2 * math.pi * vMotor / stepsOfRevolution # rad/s
        else: # Backward
            vMotor = - steps/deltaT # steps/s
            omegaLeft = 2 * math.pi * vMotor / stepsOfRevolution # rad/s
            omegaRight = 2 * math.pi * vMotor / stepsOfRevolution # rad/s
        return omegaLeft, omegaRight
class slam_and_ultrasonic:
    def __init__(self):
        # Command to control
        self.turnCount = -2
        self.close = 0
        self.turnCommand = True
        self.turnToMaxDistance = False
        self.setTargetToMaxDistanceCommand = False
        self.forwardCommand = False
        self.backwardCommand = False
        # Other parameter
        self.deltaT = 2
        self.angleOfMaxDistance = 0
        self.numOfTurn = 0
        self.orientationMax = 0
        self.remainSteps = 0
        self.stepsTurn = 20
        self.velocityToTurn = 0.5
        self.velocityToForward = 0.5
        self.velocityToBackward = 0.5
        self.stepsForward = 20
        self.stepsBackward = 20
        # Sensor data
        self.sonar0 = 0.0
        self.sonar45 = 0.0
        self.sonar90 = 0.0
        self.sonar135 = 0.0
        self.sonar180 = 0.0
        self.sonar270 = 0.0
        #self.angularData = 0.0
        # Motor data
        self.omegaLeft = 0.0
        self.omegaRight = 0.0
        self.stepsOfRevolution = 200
        # Store data
        self.allDistance360 = [] # List stores distance data when robot rotates 360 degree
        self.allAngle360 = [] # List stores angle of distance data when robot rotates 360 degree
        self.file = open("/home/ubuntu/catkin_ws/src/slam_and_ultrasonic/scripts/result.txt", "w+")
        # Position of robot
        self.pose = [0.0, 0.0, 90.0]
        # Publishers
        self.motor_params = Float32MultiArray()
        self.pub_motor = rospy.Publisher("/robot/motor", Float32MultiArray, queue_size = 100)
        # Library used 
        self.lib = slam_sonar_lib()
    def slam_control(self):
        # rospy.Subscriber("/robot/sensor/MPU6050", Float32, self.angularDataCallback)
        rospy.Subscriber("/robot/sensor/sonar0", Float32, self.sonar0Callback)
        rospy.Subscriber("/robot/sensor/sonar45", Float32, self.sonar45Callback)
        rospy.Subscriber("/robot/sensor/sonar90", Float32, self.sonar90Callback)
        rospy.Subscriber("/robot/sensor/sonar135", Float32, self.sonar135Callback)
        rospy.Subscriber("/robot/sensor/sonar180", Float32, self.sonar180Callback)
        rospy.Subscriber("/robot/sensor/sonar270", Float32, self.sonar270Callback)
        timer = rospy.Timer(rospy.Duration(self.deltaT), self.timerCallback)
        rospy.spin()
        timer.shutdown()
    def sonar0Callback(self, data):
        self.sonar0 = data.data
    def sonar45Callback(self, data):
        self.sonar45 = data.data
    def sonar90Callback(self, data):
        self.sonar90 = data.data
    def sonar135Callback(self, data):
        self.sonar135 = data.data
    def sonar180Callback(self, data):
        self.sonar180 = data.data
    def sonar270Callback(self, data):
        self.sonar270 = data.data
    def angularDataCallback(self, data):
        self.angularData = data.data
    def motorStateCallback(self, data):
        self.motorState = data.data
    def timerCallback(self, event):
        if self.turnCount < 1:
            steps = 20
            velocity = self.velocityToTurn
            self.forward(velocity,steps)
            self.turnCount += 1
        else:
            if self.turnCommand == True:
                steps = self.stepsTurn
                velocity = self.velocityToTurn
                orientation = 0 # turn right
                self.turn(velocity, orientation, steps)
                omegaLeft, omegaRight = self.lib.computeAngularVelocity(steps, orientation, self.stepsOfRevolution, self.deltaT)
                self.pose = self.lib.computeNewPose(self.pose, self.deltaT, omegaRight, omegaLeft)
                print("Current Pose: ", self.pose)
                theta = self.pose[2]
                distanceList = [self.sonar0, self.sonar90, self.sonar180, self.sonar270]
                dataPoint = self.lib.extractPoint(self.pose[0], self.pose[1], theta, distanceList)
                data = np.array(dataPoint)
                data_str = str(data)
                self.file.write(data_str)
                self.file.write("\n")
                angleList = self.lib.generateAngleList(theta)
                self.allDistance360 = self.lib.addArray(self.allDistance360, distanceList)
                self.allAngle360 = self.lib.addArray(self.allAngle360, angleList)
                print("turnCount ", self.turnCount)
                self.turnCount += 1
                if self.turnCount > 50:
                    self.turnCommand = False
                    self.turnToMaxDistance = True
                    self.setTargetToMaxDistanceCommand = True
                    self.turnCount = 1
                    self.close += 1
            else:
                if self.turnToMaxDistance == True:
                    if self.setTargetToMaxDistanceCommand == True:
                        maxDistance = max(self.allDistance360)
                        indexList = [i for i, x in enumerate(self.allDistance360) if x == maxDistance]
                        index = random.randint(0, len(indexList)-1)
                        self.angleOfMaxDistance = self.allAngle360[indexList[index]]
                        [deltaAngle, self.orientationMax] = self.lib.computeDifferent(self.pose[2], self.angleOfMaxDistance)
                        stepsToMaxDistance = self.lib.computeSteps(deltaAngle)
                        self.numOfTurn, self.remainSteps = divmod(stepsToMaxDistance, self.stepsTurn)
                        self.setTargetToMaxDistanceCommand = False
                        self.allDistance360 = [] # List stores distance data when robot rotates 360 degree
                        self.allAngle360 = []
                        print("Angle max ", self.angleOfMaxDistance)
                    if (self.numOfTurn == 0):
                        steps = self.remainSteps
                        velocity = self.velocityToTurn
                        self.turn(velocity, self.orientationMax, steps)
                        omegaLeft, omegaRight = self.lib.computeAngularVelocity(steps,self.orientationMax, self.stepsOfRevolution, self.deltaT)
                        if self.orientationMax == 0:
                            self.pose = self.lib.computeNewPose(self.pose, self.deltaT, omegaRight, omegaLeft)
                        elif self.orientationMax == 1:
                            self.pose = self.lib.computeNewPose(self.pose, self.deltaT, omegaRight, omegaLeft)
                        print("Current Pose: ", self.pose)
                        theta = self.pose[2]
                        distanceList = [self.sonar0, self.sonar90, self.sonar180, self.sonar270]
                        dataPoint = self.lib.extractPoint(self.pose[0], self.pose[1], theta, distanceList)
                        data = np.array(dataPoint)
                        data_str = str(data)
                        self.file.write(data_str)
                        self.file.write("\n")
                        self.turnToMaxDistance = False
                        self.forwardCommand = True
                    else:
                        steps = self.stepsTurn
                        velocity = self.velocityToTurn
                        self.turn(velocity, self.orientationMax, steps)
                        omegaLeft, omegaRight = self.lib.computeAngularVelocity(steps, self.orientationMax, self.stepsOfRevolution, self.deltaT)
                        if self.orientationMax == 0:
                            self.pose = self.lib.computeNewPose(self.pose, self.deltaT, omegaRight, omegaLeft)
                        elif self.orientationMax == 1:
                            self.pose = self.lib.computeNewPose(self.pose, self.deltaT, omegaRight, omegaLeft)
                        print("Current Pose: ", self.pose)
                        theta = self.pose[2]
                        distanceList = [self.sonar0, self.sonar90, self.sonar180, self.sonar270]
                        dataPoint = self.lib.extractPoint(self.pose[0], self.pose[1], theta, distanceList)
                        data = np.array(dataPoint)
                        data_str = str(data)
                        self.file.write(data_str)
                        self.file.write("\n")
                        self.numOfTurn -= 1
                if self.forwardCommand == True:
                    velocity = self.velocityToForward
                    steps = self.stepsForward
                    self.forward(velocity, steps)
                    orientation = -1 # Forward
                    omegaLeft, omegaRight = self.lib.computeAngularVelocity(steps, orientation, self.stepsOfRevolution, self.deltaT)
                    self.pose = self.lib.computeNewPose(self.pose, self.deltaT, omegaLeft, omegaRight)
                    print("Current Pose: ", self.pose)
                    theta = self.pose[2]
                    distanceList = [self.sonar0, self.sonar90, self.sonar180, self.sonar270]
                    dataPoint = self.lib.extractPoint(self.pose[0], self.pose[1], theta, distanceList)
                    data = np.array(dataPoint)
                    data_str = str(data)
                    self.file.write(data_str)
                    self.file.write("\n")
                    if self.sonar90 < 0.5:
                        self.forwardCommand = False
                        self.backwardCommand = True
                    if self.sonar45 < 0.1:
                        self.forwardCommand = False
                        self.backwardCommand = True
                    if self.sonar135 < 0.1:
                        self.forwardCommand = False
                        self.backwardCommand = True
                if self.backwardCommand == True:
                    velocity = self.velocityToBackward
                    steps = self.stepsBackward
                    self.backward(velocity, steps)
                    orientation = -2
                    omegaLeft, omegaRight = self.lib.computeAngularVelocity(steps, orientation, self.stepsOfRevolution, self.deltaT)
                    self.pose = self.lib.computeNewPose(self.pose, self.deltaT, omegaLeft, omegaRight)
                    print("Current Pose: ", self.pose)
                    theta = self.pose[2]
                    distanceList = [self.sonar0, self.sonar90, self.sonar180, self.sonar270]
                    dataPoint = self.lib.extractPoint(self.pose[0], self.pose[1], theta, distanceList)
                    data = np.array(dataPoint)
                    data_str = str(data)
                    self.file.write(data_str)
                    self.file.write("\n")
                    if self.sonar90 > 0.4:
                        if self.sonar45 > 0.4:
                            if self.sonar135 > 0.4:
                                self.backwardCommand = False
                                self.forwardCommand = False
                                self.turnCommand = True
            if self.close > 5:
                print('Done!')
                self.motor_params.data = [0, 0, 0]
                self.pub_motor.publish(self.motor_params)
                self.file.close()           
                self.turnCommand = False 
    def turn(self, velocity, orientation, steps):
        # orientation: 0 is turn right, 1 is turn left
        if orientation == 0:
            self.motor_params.data = [velocity, 0, steps]
        else:
            self.motor_params.data = [0, velocity, steps]
        self.pub_motor.publish(self.motor_params)
        
    def forward(self, velocity, steps):
        self.motor_params.data = [velocity, velocity, steps]
        self.pub_motor.publish(self.motor_params)
        
    def backward(self, velocity, steps):
        self.motor_params.data = [-velocity, -velocity, steps]
        self.pub_motor.publish(self.motor_params)
        

if __name__ == '__main__':
    print("Running")
    rospy.init_node("slam_and_ultrasonic", anonymous=False)
    slam = slam_and_ultrasonic()
    slam.slam_control()