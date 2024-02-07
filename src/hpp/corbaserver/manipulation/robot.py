#!/usr/bin/env python
# Copyright (c) 2014 CNRS
# Author: Florent Lamiraux
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

from hpp.corbaserver.manipulation import Client as ManipulationClient
from hpp.corbaserver import Client as BasicClient
from hpp.corbaserver.robot import Robot as Parent
from hpp.corbaserver.robot import StaticStabilityConstraintsFactory


class CorbaClient:
    """
    Container for corba clients to various interfaces.
    """

    def __init__(self, url=None, context="corbaserver"):
        self.basic = BasicClient(url=url, context=context)
        self.manipulation = ManipulationClient(url=url, context=context)


class Robot(Parent):
    """
    Load and handle a composite robot for manipulation planning

    A composite robot is a kinematic chain composed of several sub-kinematic
    chains rooted at an anchor joint.
    """

    def __init__(
        self,
        compositeName=None,
        robotName=None,
        rootJointType=None,
        load=True,
        client=None,
    ):
        """
        Constructor
        \\param robotName name of the first robot that is loaded now,
        \\param rootJointType type of root joint among ("freeflyer", "planar",
               "anchor"),
        \\param load whether to actually load urdf files. Set to no if you only
               want to initialize a corba client to an already initialized
               problem.
        """
        if client is None:
            client = CorbaClient()
        super(Robot, self).__init__(
            robotName=compositeName,
            rootJointType=rootJointType,
            load=False,
            client=client,
            hppcorbaClient=client.basic,
        )
        self.rootJointType = dict()
        if compositeName is None:
            load = False
        self.load = load
        self.robotNames = list()
        if robotName is None:
            if load:
                self.client.basic.robot.createRobot(self.name)
        else:
            self.loadModel(robotName, rootJointType)

    def loadModel(self, robotName, rootJointType):
        """Virtual function to load the robot model."""
        if self.load:
            self.client.basic.robot.createRobot(self.name)
        urdfFilename, srdfFilename = self.urdfSrdfFilenames()
        if self.urdfSrdfString():
            self.insertRobotModelFromString(
                robotName, rootJointType, urdfFilename, srdfFilename
            )
        else:
            self.insertRobotModel(robotName, rootJointType, urdfFilename, srdfFilename)

    def insertRobotModel(self, robotName, rootJointType, urdfName, srdfName):
        """
        Load robot model and insert it in the device

        \\param robotName key of the robot in hpp::manipulation::ProblemSolver object
            map (see hpp::manipulation::ProblemSolver::addRobot)
        \\param rootJointType type of root joint among "anchor", "freeflyer",
            "planar",
        \\param urdfName name of the urdf file
        \\param srdfName name of the srdf file

        """
        if self.load:
            self.client.manipulation.robot.insertRobotModel(
                robotName, rootJointType, urdfName, srdfName
            )
        self.robotNames.append(robotName)
        self.rootJointType[robotName] = rootJointType
        self.rebuildRanks()

    def insertRobotModelOnFrame(
        self, robotName, frameName, rootJointType, urdfName, srdfName
    ):
        """
        Insert robot model as a child of a frame of the Device

        \\param robotName key of the robot in ProblemSolver object map
               (see hpp::manipulation::ProblemSolver::addRobot)
        \\param frameName name of the existing frame
                that will the root of the added robot,
        \\param rootJointType type of root joint among "anchor", "freeflyer",
        "planar",
        \\param urdfName name of the urdf file
        \\param srdfName name of the srdf file

        """
        if self.load:
            self.client.manipulation.robot.insertRobotModelOnFrame(
                robotName, frameName, rootJointType, urdfName, srdfName
            )
        self.robotNames.append(robotName)
        self.rootJointType[robotName] = rootJointType
        self.rebuildRanks()

    def insertRobotModelFromString(
        self, robotName, rootJointType, urdfString, srdfString, frame="universe"
    ):
        """
        Same as Robot.insertRobotModel

        \\param urdfString XML string of the URDF,
        \\param srdfString XML string of the SRDF
        """
        if self.load:
            self.client.manipulation.robot.insertRobotModelOnFrameFromString(
                robotName, frame, rootJointType, urdfString, srdfString
            )
        self.robotNames.append(robotName)
        self.rootJointType[robotName] = rootJointType
        self.rebuildRanks()

    def insertRobotSRDFModel(self, robotName, srdfPath):
        """
        Load a SRDF for the robot. Several SRDF can thus be loaded for the same robot

        \\param robotName key of the robot in hpp::manipulation::Device object
               map (see hpp::manipulation::Device)
        \\param srdfPath path to srdf file (can start with "package://")
        """
        if self.load:
            self.client.manipulation.robot.insertRobotSRDFModel(robotName, srdfPath)

    def insertHumanoidModel(self, robotName, rootJointType, urdfName, srdfName):
        """
        Load humanoid robot model and insert it in the device

        \\param robotName key of the robot in hpp::manipulation::ProblemSolver object
               map (see hpp::manipulation::ProblemSolver::addRobot)
        \\param rootJointType type of root joint among "anchor", "freeflyer",
               "planar",
        \\param urdfName name of the urdf file
        \\param srdfName name of the srdf file
        """
        if self.load:
            self.client.manipulation.robot.insertHumanoidModel(
                robotName, rootJointType, urdfName, srdfName
            )
        self.robotNames.append(robotName)
        self.rootJointType[robotName] = rootJointType
        self.rebuildRanks()

    def insertHumanoidModelFromString(
        self, robotName, rootJointType, urdfString, srdfString
    ):
        """
        Same as Robot.insertHumanoidModel

        \\param urdfString XML string of the URDF,
        \\param srdfString XML string of the SRDF
        """
        if self.load:
            self.client.manipulation.robot.insertHumanoidModelFromString(
                robotName, rootJointType, urdfString, srdfString
            )
        self.robotNames.append(robotName)
        self.rootJointType[robotName] = rootJointType
        self.rebuildRanks()

    def loadHumanoidModel(self, robotName, rootJointType, urdfName, srdfName):
        self.insertHumanoidModel(robotName, rootJointType, urdfName, srdfName)

    def loadEnvironmentModel(self, urdfName, srdfName, envName):
        """
        Load environment model and store in local map.
        Contact surfaces are build from the corresping srdf file.
        See hpp-manipulation-urdf for more details about contact surface
        specifications.

        \\param envName key of the object in ProblemSolver object map
               (see hpp::manipulation::ProblemSolver::addRobot)
        \\param urdfName name of the urdf file,
        \\param srdfName name of the srdf file.
        """
        if self.load:
            self.client.manipulation.robot.loadEnvironmentModel(
                urdfName, srdfName, envName
            )
        self.rootJointType[envName] = "Anchor"

    # # \name Joints
    # \{

    def setRootJointPosition(self, robotName, position):
        """
        Set the position of root joint of a robot in world frame
        \\param robotName key of the robot in ProblemSolver object map.
        \\param position constant position of the root joint in world frame in
               initial configuration.
        """
        return self.client.manipulation.robot.setRootJointPosition(robotName, position)

    # # \}

    # # \name Bodies
    #  \{

    def getGripperPositionInJoint(self, gripperName):
        """
        Return the joint name in which a gripper is and the position relatively
        to the joint
        """
        return self.client.manipulation.robot.getGripperPositionInJoint(gripperName)

    def getHandlePositionInJoint(self, handleName):
        """
        Return the joint name in which a handle is and the position relatively
        to the joint
        """
        return self.client.manipulation.robot.getHandlePositionInJoint(handleName)

    def setHandlePositionInJoint (self, handleName, position):
        """Set handle position in joint frame"""
        return self.client.manipulation.robot.setHandlePositionInJoint\
            (handleName, position)

    # # \}


class HumanoidRobot(Robot, StaticStabilityConstraintsFactory):
    def __init__(
        self,
        compositeName=None,
        robotName=None,
        rootJointType=None,
        load=True,
        client=None,
    ):
        """
        Constructor
        \\param compositeName name of the composite robot that will be built later,
        \\param robotName name of the first robot that is loaded now,
        \\param rootJointType type of root joint among ("freeflyer", "planar",
               "anchor"),
        """
        Robot.__init__(self, compositeName, robotName, rootJointType, load, client)

    def loadModel(self, robotName, rootJointType):
        if self.load:
            self.client.basic.robot.createRobot(self.name)
        urdfFilename, srdfFilename = self.urdfSrdfFilenames()
        if self.urdfSrdfString():
            self.insertHumanoidModelFromString(
                robotName, rootJointType, urdfFilename, srdfFilename
            )
        else:
            self.insertHumanoidModel(
                robotName, rootJointType, urdfFilename, srdfFilename
            )
