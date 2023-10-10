# January 6,2022
import open3d as o3d
import numpy as np
np.set_printoptions(suppress=True)

import matplotlib.pyplot as plt
from numpy import linalg, mat
from numpy import arctan
import cmath
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi
import math
import os
import controller
from controller import Supervisor
from math import cos, sin, atan2, acos, asin, sqrt, pi

from spatialmath import SE3, Twist3
import time
import copy

class InverseKinematics():
    def __init__(self):
        self.d1,self.a2,self.a3,self.d4,self.d5,self.d6 = 0.1625,-0.425,-0.3922,0.1333,0.0997,0.0996
        self.d = np.matrix([self.d1, 0, 0, self.d4, self.d5, self.d6])
        self.a = np.matrix([0, self.a2, self.a3, 0, 0, 0])
        self.alph = np.matrix([pi / 2, 0, 0, pi / 2, -pi / 2, 0])

    def AH(self,n, th,c):
        # n: the link
        # th: vector of angles of each joint?
        # d=
        T_a = np.matrix(np.identity(4), copy=False)

        T_a[0, 3] = self.a[0, n - 1]
        T_d = np.matrix(np.identity(4), copy=False)
        T_d[2, 3] = self.d[0, n - 1]

        Rzt = np.matrix([[cos(th[n - 1, c]), -sin(th[n - 1, c]), 0, 0],
                   [sin(th[n - 1, c]), cos(th[n - 1, c]), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]], copy=False)

        Rxa = np.matrix([[1, 0, 0, 0],
                   [0, cos(self.alph[0, n - 1]), -sin(self.alph[0, n - 1]), 0],
                   [0, sin(self.alph[0, n - 1]), cos(self.alph[0, n - 1]), 0],
                   [0, 0, 0, 1]], copy=False)

        A_i = T_d * Rzt * T_a * Rxa
        return A_i

    def HTrans(self,th, c):
        # th is a 6 x 8 matrix where each column is a different solution
        # c is the index of the column of theta used
        A_1 = self.AH(1, th, c)
        A_2 = self.AH(2, th, c)
        A_3 = self.AH(3, th, c)
        A_4 = self.AH(4, th, c)
        A_5 = self.AH(5, th, c)
        A_6 = self.AH(6, th, c)
        T_06 = A_1 * A_2 * A_3 * A_4 * A_5 * A_6
        return T_06

    def invKine(self,desired_pos):
        # Returns 6 by 8 matrix where each column is a different solution
        # desired_pos is the homogenuous transform for some (x,y,z) position
        th = np.matrix(np.zeros((6, 8)))
        P_05 = desired_pos * np.matrix([0, 0, -self.d6, 1]).T - np.matrix([0, 0, 0, 1]).T

        # **** theta1 ****
        psi = atan2(P_05[2 - 1, 0], P_05[1 - 1, 0])
        phi = acos(self.d4 / sqrt(P_05[2 - 1, 0] * P_05[2 - 1, 0] + P_05[1 - 1, 0] * P_05[1 - 1, 0]))
        # The two solutions for theta1 correspond to the shoulder
        # being either left or right
        th[0, 0:4] = pi / 2 + psi + phi
        th[0, 4:8] = pi / 2 + psi - phi
        th = th.real
        # **** theta5 ****
        cl = [0, 4]  # wrist up or down
        for i in range(0, len(cl)):
            c = cl[i]
            T_10 = linalg.inv(self.AH(1, th, c))
            T_16 = T_10 * desired_pos
            th[4, c:c + 2] = + acos((T_16[2, 3] - self.d4) / self.d6)
            th[4, c + 2:c + 4] = - acos((T_16[2, 3] - self.d4) / self.d6)
        th = th.real

        # **** theta6 ****
        # theta6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.
        cl = [0, 2, 4, 6]
        for i in range(0, len(cl)):
            c = cl[i]
            T_10 = linalg.inv(self.AH(1, th, c))
            T_16 = linalg.inv(T_10 * desired_pos)
            th[5, c:c + 2] = atan2((-T_16[1, 2] / sin(th[4, c])), (T_16[0, 2] / sin(th[4, c])))
        th = th.real

        # **** theta3 ****
        cl = [0, 2, 4, 6]
        for i in range(0, len(cl)):
            c = cl[i]
            T_10 = linalg.inv(self.AH(1, th, c))
            T_65 = self.AH(6, th, c)
            T_54 = self.AH(5, th, c)
            T_14 = (T_10 * desired_pos) * linalg.inv(T_54 * T_65)
            P_13 = T_14 * np.matrix([0, -self.d4, 0, 1]).T - np.matrix([0, 0, 0, 1]).T
            t3 = cmath.acos((linalg.norm(P_13) ** 2 - self.a2 ** 2 - self.a3 ** 2) / (2 * self.a2 * self.a3))  # norm ?
            th[2, c] = t3.real
            th[2, c + 1] = -t3.real

        # **** theta2 and theta 4 ****
        cl = [0, 1, 2, 3, 4, 5, 6, 7]
        for i in range(0, len(cl)):
            c = cl[i]
            T_10 = linalg.inv(self.AH(1, th, c))
            T_65 = linalg.inv(self.AH(6, th, c))
            T_54 = linalg.inv(self.AH(5, th, c))
            T_14 = (T_10 * desired_pos) * T_65 * T_54
            P_13 = T_14 * np.matrix([0, -self.d4, 0, 1]).T - np.matrix([0, 0, 0, 1]).T

            # theta 2
            th[1, c] = -atan2(P_13[1], -P_13[0]) + asin(self.a3 * sin(th[2, c]) / linalg.norm(P_13))

            # theta 4
            T_32 = linalg.inv(self.AH(3, th, c))
            T_21= linalg.inv(self.AH(2, th, c))
            T_34 = T_32 * T_21 * T_14
            th[3, c] = atan2(T_34[1, 0], T_34[0, 0])
        th = th.real
        return th


class Block():
    def __init__(self,  name, pcd):
        self.blockPCD = pcd
        self.name = name
        # Removes outlier points by fitting block into largest cluster
        self.clusterBlockPCD()
        self.blockAABB = self.blockPCD.get_axis_aligned_bounding_box()
        self.robotCoordsDuringImage = np.array(
            [-0.67743, -0.13328, -0.008594])  # the coordinates of the gripper when it took the images
        self.camCoords = self.getCameraCoordinates()
        self.robotCoords = self.getRobotCoordinates()
        self.currentCamCoords = self.camCoords

    def clusterBlockPCD(self):
        # modifies block PCD to only contain points in the largest cluster found with DBScan
        # eps found experimentally
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Error) as cm:
            # eps is radius
            # rejects points that are too small
            labels = np.array(self.blockPCD.cluster_dbscan(eps=0.013, min_points=20, print_progress=False))
        max_label = labels.max()
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        clusters = {}
        for i in range(0, max_label + 1):
            clusters[i] = []

        for i in range(0, len(labels)):
            if labels[i] != -1:
                clusters[labels[i]].append(i)

        clusterPCDs = []
        for clusterLabel in clusters:
            clusterPCD = self.blockPCD.select_by_index(clusters[clusterLabel])
            clusterPCDs.append(clusterPCD)
        self.blockPCD = clusterPCDs[0]

    def getCameraCoordinates(self):
        # returns the (x,y,z) coordinates in the camera's coordinates frame of where the gripper should be placed
        # center of front-facing axis-aligned bounding box
        x, y = self.blockAABB.get_center()[0:2]
        z = self.blockAABB.get_min_bound()[2]
        return (x, y, z)

    def getRobotCoordinates(self):
        xGripper, yGripper, zGripper = self.robotCoordsDuringImage
        xCam, yCam, zCam = self.camCoords
        # xGripper_Goal = xGripper - zCam
        # yGripper_Goal = yGripper + xCam
        # zGripper_Goal = zGripper - yCam
        # changed to deltas
        deltaXGripper = -zCam
        deltaYGripper = xCam
        deltaZGripper = -yCam
        return np.array([deltaXGripper, deltaYGripper, deltaZGripper])

    def move(self, goalCamCoords):
        self.currentCamCoords = goalCamCoords

def showImages(images):
    for image in images:
        plt.imshow(image)
        plt.show()

def getBlocksFromImages(images):
    # returns block objects given images = regImage,segImage,depthImage
    # takes in regular image, segmented image from webots, and depth image from range-finder
    regImage, segImage, depthImage = images
    # block pixels set to 255
    redMask = np.dot(segImage, [1, 0, 0, 0]).astype('uint16')
    redMask[redMask == 255] = 1  # mask where red block has pixel value 1
    greenMask = np.dot(segImage, [0, 1, 0, 0]).astype('uint16')
    greenMask[greenMask == 255] = 1
    blueMask = np.dot(segImage, [0, 0, 1, 0]).astype('uint16')
    blueMask[blueMask == 255] = 1

    depthImage[depthImage == math.inf] = 0  # far away values set to 0 distance, =

    # mask the depth image using the monochromatic segmented image so that only blocks have defined values
    redDepthImage = np.multiply(depthImage, redMask)
    greenDepthImage = np.multiply(depthImage, greenMask)
    blueDepthImage = np.multiply(depthImage, blueMask)

    # depthImageSeg = np.multiply(depthImage,mask)
    images.append(redDepthImage)
    images.append(greenDepthImage)
    images.append(blueDepthImage)
    # SEGMENT PCD INTO RED,GREEN,BLUE BLOCKS
    redRGDB_Image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(regImage),
        o3d.geometry.Image(np.array(redDepthImage).astype('uint16')),
        convert_rgb_to_intensity=False,
        depth_scale=1000.0)

    greenRGDB_Image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(regImage),
        o3d.geometry.Image(np.array(greenDepthImage).astype('uint16')),
        convert_rgb_to_intensity=False,
        depth_scale=1000.0)

    blueRGBD_Image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(regImage),
        o3d.geometry.Image(np.array(blueDepthImage).astype('uint16')),
        convert_rgb_to_intensity=False,
        depth_scale=1000.0)

    redPCD = o3d.geometry.PointCloud.create_from_rgbd_image(
        redRGDB_Image,
        o3d.camera.PinholeCameraIntrinsic(320, 240, 320, 240, 160, 120),
        project_valid_depth_only=True
    )
    greenPCD = o3d.geometry.PointCloud.create_from_rgbd_image(
        greenRGDB_Image,
        o3d.camera.PinholeCameraIntrinsic(320, 240, 320, 240, 160, 120),
        project_valid_depth_only=True
    )
    bluePCD = o3d.geometry.PointCloud.create_from_rgbd_image(
        blueRGBD_Image,
        o3d.camera.PinholeCameraIntrinsic(320, 240, 320, 240, 160, 120),
        project_valid_depth_only=True
    )

    # flip point clouds since they start upside down
    # redPCD.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
    # greenPCD.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
    # bluePCD.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
    redPCD.paint_uniform_color([1, 0, 0])
    greenPCD.paint_uniform_color([0, 1, 0])
    bluePCD.paint_uniform_color([0, 0, 1])
    redBlock = Block("redBlock", redPCD)
    # greenBlock = Block("greenBlock", greenPCD)
    # blueBlock = Block("blueBlock", bluePCD)
    print("Displaying image in open3d")
    o3d.visualization.draw([redPCD])
    print("Done")
    # showImages(images)
    return (None,None,None)
    # return (redBlock, greenBlock, blueBlock)


def displayWorld(blocks):
    coordFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    geometry = [coordFrame]
    for block in blocks:
        geometry.append(block.blockPCD)
        geometry.append(block.blockAABB)
        blockCoordFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=block.camCoords)
        geometry.append(blockCoordFrame)
        '''
        print(f"{block.name}")
        deltas = ["dx","dy","dz"]
        for i in range(0,len(block.robotCoords)):
            print(f"{deltas[i]}: {block.robotCoords[i]}")

        print(f"{block.name}\nCam Coordinates: {block.camCoords}")
        '''
        # print(f"Robot Coordinates: {block.robotCoords}")
    o3d.visualization.draw(geometry)


class ArmController():
    def __init__(self,enableSensing=True):
        # enableSensing toggles whether camera/rangefinder should be enabled, turning off sensing speeds up simulation
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep()) # sampling period for sensors
        self.camera = self.robot.getDevice('camera')
        if enableSensing:
            self.camera.enable(self.timestep)
            self.camera.recognitionEnable(self.timestep)
            self.camera.enableRecognitionSegmentation()

        self.rangeFinder = self.robot.getDevice("range-finder")
        if enableSensing:
            self.rangeFinder.enable(self.timestep)
        self.plane =  [0, 0, 1, 0.05]
        self.IK = InverseKinematics()
        self.motors = []
        self.armSensors = []
        self.handMotors = []
        armMotorNames = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
                         "wrist_3_joint"]
        armMotorSensorNames = ["shoulder_pan_joint_sensor", "shoulder_lift_joint_sensor", "elbow_joint_sensor",
                               "wrist_1_joint_sensor", "wrist_2_joint_sensor",
                               "wrist_3_joint_sensor"]
        handMotorNames = ["finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1"]

        self.speed = 1.0
        for i in range(0, len(armMotorNames)):
            motor = self.robot.getDevice(armMotorNames[i])
            motor.setVelocity(self.speed)
            self.motors.append(motor)
            armSensor = self.robot.getDevice(armMotorSensorNames[i])
            armSensor.enable(self.timestep)
            self.armSensors.append(armSensor)

        for i in range(0, len(handMotorNames)):
            self.handMotors.append(self.robot.getDevice(handMotorNames[i]))
            self.handMotors[i].setVelocity(0.5)

        self.robot.step(self.timestep) # Initial time step so sensors values are not null
        self.homePose = self.getPose()

    def moveToHomeState(self):
        print("Moving to Home State")

        homePoseAngles = np.array([[0.00001454],[-1.29671905],[2.68977494],[-1.39291287],[1.57080523],[0.00000216]])
        # homePoseAngles = np.array([[0], [-1.382], [-1.13], [-2.2], [1.63], [0]])
        # homePoseAngles = np.array([[0], [-1.382], [-1.13], [-2.2], [1.63], [pi/3]])
        self.setJointAngles(homePoseAngles)
        for i in range(0, 300):
            self.robot.step(self.timestep)

    # Returns true if a given set of joint angles is above the plane
    def isJointPosSafe(self,joints, plane):
        # joints is a 6 x 1 vector of joint angles
        # plane is a 1 x 4 list of coefficients a,b,c,d
        T_joints = []
        for i in range(0,6):
            T_joints.append(self.IK.AH(i+1, joints, 0))

        # list of h-transforms for each joint
        T = []
        for i in range(0,6):
            T.append(T_joints[0])
            for j in range(1, i + 1):
                T[i] = T[i] * T_joints[j]

        # check if each point is above the plane
        for i in range(0,len(T)):
            jointCoordinate = T[i][0:,3].flatten().tolist()[0]
            # check if point is above the plane (ax + by + cz + d = 0 ))
            # print((plane[0]*jointCoordinate[0]) + (plane[1]*jointCoordinate[1]) + (plane[2] * jointCoordinate[2]))
            if (plane[0]*jointCoordinate[0]) + (plane[1]*jointCoordinate[1]) + (plane[2] * jointCoordinate[2]) < plane[3]:
                return False
        return True

    # returns a 6 x 1 Numpy Array of joint angles
    def getJointPositions(self):
        thetas = np.zeros((6, 1))
        for j in range(0, 6):
            thetas[j, 0] = self.armSensors[j].getValue()
        return thetas

    def setJointAngles(self,thetas):
        # thetas is a 6 x 1 vector of joint angles
        for i in range(0, 6):
            theta_i = thetas[i]
            # print(f"Motor {i} Min,Max Positions{self.motors[i].getMinPosition()},{self.motors[i].getMaxPosition()}")
            self.motors[i].setPosition(float(theta_i))

    def moveJ(self,T_06):
        # T_06 is the 4 x 4 homogenuous transform for position and rotation from the grippers coordinate frame to the base's coordinate frame
        jointAngleSolutions = self.IK.invKine(T_06)

        # Reject all joint positions that lead to an unsafe position
        # Uses the first safe set of angles
        solutionFound = False
        for col in range(0, 8):
            sol = jointAngleSolutions[0:6, col]
            if self.isJointPosSafe(sol, self.plane):
                solutionFound = True
                break

        # if there are no safe configuration
        if solutionFound == False:
            raise Exception(f"No safe joint configuration for pose {T_06}")

        timeRequired = max([(sol[i] - self.getJointPositions()[i])/(self.motors[i].getMaxVelocity()/5) for i in range(0,6)])
        # Below calc is faked, we don't really know the angular distance, assume max distance and min velocity
        timeStepsRequired = int(timeRequired / (self.timestep / 1000)) + 1
        '''
        This doesn't really work because we need to precompute the joint range required for all joints which is not done yet to find time required
        let's assume max angular distance 180 degrees for all joints
        angularDistanceRequired = (2*math.pi) - abs(startAngle - goalAngle) # rad
        angularVelocity = self.motors[row].getVelocity()  # rad/s
        timeRequired = angularDistanceRequired/angularVelocity # s
        timeStepsRequired = int(timeRequired / (self.timestep / 1000)) + 1
        '''

        # current joint angles
        jointPositions = self.getJointPositions()
        # jointSequences contains each of the sequences of joint angles for each joint i in a numpy array where jointSequences[i] 
        # describes a sequence of angles [theta_1,theta_2,...,theta_n] that are each set incrementally for joint i at each of the  timestep   
        # Each sequence in jointSequences contains timeStepsRequired elements where timeStepsRequired is found based on the amount of time required to 
        # sweep through the sequence that takes the most time  
        jointSequences = []
        for row in range(0,6):
            startAngle = jointPositions[row,0]
            goalAngle = sol[row,0]

            # First convert both start and goal angles to positive values
            '''
            if startAngle < 0:
                startAngle = (2*math.pi) + startAngle
            if goalAngle < 0:
                goalAngle = (2*math.pi) + goalAngle
            '''
            if row==0:
                print(f"startAngle:{startAngle}")
                print(f"goalAngle:{goalAngle}")

            # If |startAngle - goalAngle| > 180 than it's a shorter angular distance to cross over the line of 0/360 degrees
            # if abs(startAngle - goalAngle) > np.radians(180):
                
            '''
            if abs(startAngle - goalAngle) > np.radians(180):
                print("Taking short route")
                # if startAngle < goalAngle move in negative direction to 0 from startAngle and then to -(360-goalAngle)
                if startAngle < goalAngle:
                    jointSequence = np.linspace(startAngle,-(2*math.pi-goalAngle),timeStepsRequired)

                # Otherwise, if startAngle > goalAngle move in positive direction to 360 from startAngle and then to 360 + goalAngle
                else:
                    jointSequence = np.linspace(startAngle,(2*math.pi) + goalAngle,timeStepsRequired)
            # Otherwise, move on the number line from startAngle to goalAngle
            else:
                jointSequence = np.linspace(startAngle,goalAngle,timeStepsRequired)
            '''
            jointSequence = np.linspace(startAngle, goalAngle, timeStepsRequired)
            if row == 0:
                print(f"sequenceLength: {len(jointSequence)}")
            jointSequences.append(jointSequence)

        '''
        # Discretize the joint angles into the appropriate number of bins 
        theta0_range = np.linspace(jointPositions[0, 0], sol[0, 0], timeStepsRequired)
        theta1_range = np.linspace(jointPositions[1, 0], sol[1, 0], timeStepsRequired)
        theta2_range = np.linspace(jointPositions[2, 0], sol[2, 0], timeStepsRequired)
        theta3_range = np.linspace(jointPositions[3, 0], sol[3, 0], timeStepsRequired)
        theta4_range = np.linspace(jointPositions[4, 0], sol[4, 0], timeStepsRequired)
        theta5_range = np.linspace(jointPositions[5, 0], sol[5, 0], timeStepsRequired)
        '''
        # Incrementally move the joints there
        # note should debug additional steps here
        for i in range(0, timeStepsRequired):
            thetas = np.mat([jointSequences[0][i], jointSequences[1][i], jointSequences[2][i], jointSequences[3][i],jointSequences[4][i],
                             jointSequences[5][i]]).T
            self.setJointAngles(thetas)
            self.robot.step(self.timestep)

    def moveL(self,T_06):
        # T_06 is the 4 x 4 homogenuous transform for position and rotation from the grippers coordinate frame located at the goal to the base's coordinate frame
        # Use the forward kinematics to calculate the current position of the robot
        T_06_init = self.getPose()
        # initOrientation = T_06_init[0:3, 0:3]
        goalOrientation = T_06[0:3, 0:3]
        gripperFrameOrigin = T_06_init[0:4, 3]
        x, y, z = gripperFrameOrigin.item((0,0)), gripperFrameOrigin.item((1,0)), gripperFrameOrigin.item((2,0))
        # Determine the Euclidian distance between the current and the desired position of the robot
        goalX, goalY, goalZ = T_06.item((0, 3)), T_06.item((1, 3)), T_06.item((2, 3))
        dist = linalg.norm(np.array([goalX,goalY,goalZ])-np.array([x,y,z])) # distance in meters

        # Assume a maximum Cartesian speed of the robot, e.g. 1cm/s, and generate a list of waypoints from the current to the desired position of the robot
        # fix the orientation of the robot and vary the position
        maxSpeed = 100  # in cm/s
        timeRequired = (dist * 100) / maxSpeed
        timeStepsRequired = int(timeRequired / (self.timestep / 1000)) + 1
        xRange = np.linspace(x, goalX, timeStepsRequired)
        yRange = np.linspace(y, goalY, timeStepsRequired)
        zRange = np.linspace(z, goalZ, timeStepsRequired)
        wayPointTransforms = []
        for i in range(1, len(xRange)):
            goalX, goalY, goalZ = xRange[i], yRange[i], zRange[i]
            # fix the pose of the gripper but vary the position of it's center
            wayPointTransform = np.matrix(np.zeros((4, 4)))
            wayPointTransform[3, 3] = 1
            wayPointTransform[0:3, 0:3] = goalOrientation
            wayPointTransform[0, 3] = goalX
            wayPointTransform[1, 3] = goalY
            wayPointTransform[2, 3] = goalZ
            wayPointTransforms.append(wayPointTransform)

        # Create a control loop that calls MoveJ to move from point to point. For smoother motion, you can pull the next waypoint, once the robot gets reasonably close to the next waypoint.
        for transform in wayPointTransforms:
            # move to pose given by transform
            self.moveJ(transform)
            # print(f"Error {np.linalg.norm(self.getXYZ() - np.array((goalX,goalY,goalZ)))}")

    def closeGripper(self):
        print("Closing Gripper")
        # double torque of middle finger
        # self.handMotors[2].setAvailableTorque(200)
        # handMotors ordered as topFinger,bottomFinger,middleFinger
        for i in range(0, len(self.handMotors)):
            self.handMotors[i].setPosition(float(np.radians(20)))
        for i in range(0, 500):
            self.robot.step(self.timestep)

    def openGripper(self):
        print("Opening Gripper")
        for i in range(0, len(self.handMotors)):
            self.handMotors[i].setPosition(self.handMotors[i].getMinPosition())
        for i in range(0, 500):
            self.robot.step(self.timestep)

    def getPose(self):
        # returns the coordinate transform in the robot's frame associated with the current pose
        jointPositions = self.getJointPositions()
        T_06 = self.IK.HTrans(jointPositions,0)
        return np.matrix(T_06)

    def getXYZ(self):
        currentPose = self.getPose()
        x,y,z = currentPose[0, 3], currentPose[1, 3], currentPose[2, 3]
        return np.array([x,y,z])

    def getGoalPose(self,deltas):
        # deltas is tuple of (dX,dY,dZ)
        dX,dY,dZ = deltas
        # returns a new homogenous transorm with x,y,z incremented by dX,dY,dZ
        currentPose = self.getPose()
        # print(f"Current Pose: \n {currentPose}")
        goalPose = np.array(currentPose)
        # print(f"dX: {dX}")
        # print(f"dY: {dY}")
        # print(f"dZ: {dZ}")
        goalPose[0,3] += dX
        goalPose[1,3] += dY
        goalPose[2,3] += dZ
        # print(f"Goal Pose: {goalPose}")
        return goalPose

    def getGoalPoseHome(self,deltas):
        # returns a new homogenous transorm with x,y,z incremented by dX,dY,dZ
        # relative to home position
        dX, dY, dZ = deltas
        homePose = copy.deepcopy(self.homePose)
        homePose[0, 3] += dX
        homePose[1, 3] += dY
        homePose[2, 3] += dZ
        return homePose

    def getCameraImage(self):
        imageWidth,imageHeight = self.camera.getWidth(),self.camera.getHeight()
        image1D = self.camera.getImage()
        image = np.frombuffer(image1D, np.uint8).reshape((imageHeight,imageWidth,4))
        return image

    def getSegmentedImage(self):
        # returns segemented image as np array with shape (320,240,4)
        imageWidth,imageHeight = self.camera.getWidth(),self.camera.getHeight()
        image1DSeg = self.camera.getRecognitionSegmentationImage()
        imageSeg = np.frombuffer(image1DSeg, np.uint8).reshape((imageHeight, imageWidth, 4))
        return imageSeg

    def getDepthImage(self):
        depthImage = np.array(self.rangeFinder.getRangeImageArray())
        return depthImage

    def collectSampleData(self):
        cameraImage = self.getCameraImage()
        segImage = self.getSegmentedImage()
        depthImage = self.getDepthImage()
        plt.imshow(cameraImage)
        plt.show()
        plt.imshow(depthImage)
        plt.show()
        plt.imshow(segImage)
        plt.show()
        np.save("depth_image_adjusted", depthImage)
        np.save("seg_image_adjusted", segImage)
        np.save("reg_image_adjusted", cameraImage)

    def moveBlock(self,blockPos=(-0.55,0.3,0.12),pickup=True):
        # pickup (boolean) toggles whether to open or close gripper upon reaching location
        # pickup=False implies object is dropped
        # blockPos = (dX,dY,dZ) of block relative to home position
        blockX,blockY,blockZ = blockPos
        # blockX,blockY,blockZ = -0.15,0.3,0.2
        # move left/right (+y) to block
        pose1 = self.getGoalPoseHome((0,blockY,0))
        self.moveL(pose1)

        # move up/down (+z) to block
        pose2 = self.getGoalPoseHome((0,blockY,blockZ))
        self.moveL(pose2)

        # move forwards (-x) to block
        pose3 = self.getGoalPoseHome((blockX, blockY, blockZ))
        self.moveL(pose3)

        print("At block")
        print("Close gripper")
        # close gripper here
        if pickup:
            self.closeGripper()
        else:
            self.openGripper()

        # rise slightly to prevent friction
        pose4 = self.getGoalPose((0,0,0.05))
        self.moveL(pose4)

        # move backwards (+x) from block
        pose5 = self.getGoalPoseHome((0,blockY,blockZ))
        self.moveL(pose5)

        # move up/down (-z) from block
        pose6 = self.getGoalPoseHome((0,blockY,0))
        self.moveL(pose6)

        # move left/right (-y) to home
        pose7 =self.getGoalPoseHome((0,0,0))
        self.moveL(pose7)
        print("At home position")
        # close gripper

        # self.openGripper()
        # self.closeGripper()

    def runAnalysis(self):
        regImage, segImage, depthImage = self.getCameraImage(),self.getSegmentedImage(),self.getDepthImage()
        for image in (regImage,segImage,depthImage):
            plt.imshow(image)
            plt.show()
        # blocks = getBlocksFromImages([regImage, segImage, depthImage])

    def runRoutine(self):
        print("Running routine specified in ArmController.runRoutine()")
        print(f"Initial Arm Angles {np.degrees(self.getJointPositions())}")
        self.moveToHomeState()
        # self.runAnalysis()
        blueBlock = -0.55,0.3,0.12
        self.moveBlock(blueBlock)
        # droppoff location should be slightly higher due to block below
        dropoff = list(blueBlock)
        dropoff[2] += 0.05
        self.moveBlock(dropoff,pickup=False)
        

    def wait(self,seconds):
        t0=time.time()
        for i in range(1,int(seconds*1000),self.timestep):
            self.robot.step(self.timestep)
        print(f"Waited for {time.time()-t0}")


armController = ArmController(enableSensing=True)
armController.runRoutine()