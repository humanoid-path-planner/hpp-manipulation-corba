// Copyright (c) 2012 CNRS
// Author: Florent Lamiraux, Joseph Mirabel
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include "robot.impl.hh"

#include <hpp/corbaserver/manipulation/server.hh>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/handle.hh>
#include <hpp/manipulation/srdf/util.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/humanoid-robot.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>
#include <pinocchio/multibody/model.hpp>

#include "tools.hh"

namespace hpp {
namespace manipulation {
namespace impl {
namespace {
using core::Container;
using pinocchio::Gripper;

DevicePtr_t getOrCreateRobot(ProblemSolverPtr_t p,
                             const std::string& name = "Robot") {
  DevicePtr_t r = p->robot();
  if (r) return r;
  pinocchio::DevicePtr_t robot(p->createRobot(name));
  assert(HPP_DYNAMIC_PTR_CAST(Device, robot));
  p->robot(robot);
  return HPP_STATIC_PTR_CAST(Device, robot);
}

JointPtr_t getJointByBodyNameOrThrow(ProblemSolverPtr_t p,
                                     const std::string& n) {
  DevicePtr_t r = getRobotOrThrow(p);
  JointPtr_t j = r->getJointByBodyName(n);
  return j;
}

template <typename GripperOrHandle>
GripperOrHandle copy(const GripperOrHandle& in, const DevicePtr_t& device,
                     const std::string& p);

template <>
GripperPtr_t copy(const GripperPtr_t& in, const DevicePtr_t& device,
                  const std::string& p) {
  Transform3f position = (in->joint() ? in->joint()->currentTransformation() *
                                            in->objectPositionInJoint()
                                      : in->objectPositionInJoint());

  pinocchio::Model& model = device->model();
  const std::string name = p + in->name();
  if (model.existFrame(name))
    throw std::invalid_argument("Could not add the gripper because a frame \'" +
                                name + "\" already exists.");
  model.addFrame(::pinocchio::Frame(name, model.getJointId("universe"),
                                    model.getFrameId("universe"), position,
                                    ::pinocchio::OP_FRAME));

  GripperPtr_t out = Gripper::create(name, device);
  out->clearance(in->clearance());
  return out;
}

template <>
HandlePtr_t copy(const HandlePtr_t& in, const DevicePtr_t& device,
                 const std::string& p) {
  Transform3f position =
      (in->joint() ? in->joint()->currentTransformation() * in->localPosition()
                   : in->localPosition());

  HandlePtr_t out =
      Handle::create(p + in->name(), position, device, JointPtr_t());
  out->clearance(in->clearance());
  return out;
}

template <typename Object>
void copy(const Container<Object>& from, Container<Object>& to,
          const DevicePtr_t& d, const std::string& prefix) {
  typedef Container<Object> Container_t;
  for (typename Container_t::const_iterator it = from.map.begin();
       it != from.map.end(); it++) {
    Object obj = copy<Object>(it->second, d, prefix);
    to.add(obj->name(), obj);
  }
}
}  // namespace

Robot::Robot() : server_(0x0) {}

ProblemSolverPtr_t Robot::problemSolver() { return server_->problemSolver(); }

void Robot::insertRobotModel(const char* robotName, const char* rootJointType,
                             const char* urdfName, const char* srdfName) {
  insertRobotModelOnFrame(robotName, "universe", rootJointType, urdfName,
                          srdfName);
}

void Robot::insertRobotModelOnFrame(const char* robotName,
                                    const char* frameName,
                                    const char* rootJointType,
                                    const char* urdfName,
                                    const char* srdfName) {
  try {
    DevicePtr_t robot = getOrCreateRobot(problemSolver());
    if (robot->robotFrames(robotName).size() > 0)
      HPP_THROW(std::invalid_argument,
                "A robot named " << robotName << " already exists");
    if (!robot->model().existFrame(frameName))
      HPP_THROW(std::invalid_argument, "No frame named " << frameName << ".");
    pinocchio::FrameIndex frame = robot->model().getFrameId(frameName);
    pinocchio::urdf::loadModel(robot, frame, robotName, rootJointType, urdfName,
                               srdfName);
    if (!std::string(srdfName).empty()) {
      srdf::loadModelFromFile(robot, robotName, srdfName);
    }
    problemSolver()->resetProblem();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Robot::insertRobotModelFromString(const char* robotName,
                                       const char* rootJointType,
                                       const char* urdfString,
                                       const char* srdfString) {
  insertRobotModelOnFrameFromString(robotName, "universe", rootJointType,
                                    urdfString, srdfString);
}

void Robot::insertRobotModelOnFrameFromString(const char* robotName,
                                              const char* frameName,
                                              const char* rootJointType,
                                              const char* urdfString,
                                              const char* srdfString) {
  try {
    DevicePtr_t robot = getOrCreateRobot(problemSolver());
    if (robot->robotFrames(robotName).size() > 0)
      HPP_THROW(std::invalid_argument,
                "A robot named " << robotName << " already exists");
    if (!robot->model().existFrame(frameName))
      HPP_THROW(std::invalid_argument, "No frame named " << frameName << ".");
    pinocchio::FrameIndex frame = robot->model().getFrameId(frameName);

    pinocchio::urdf::loadModelFromString(robot, frame, robotName, rootJointType,
                                         urdfString, srdfString);
    srdf::loadModelFromXML(robot, robotName, srdfString);
    problemSolver()->resetProblem();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Robot::insertRobotSRDFModel(const char* robotName, const char* srdfPath) {
  try {
    DevicePtr_t robot = getOrCreateRobot(problemSolver());
    srdf::loadModelFromFile(robot, robotName, srdfPath);
    hpp::pinocchio::urdf::loadSRDFModel(robot, robotName, srdfPath);
    problemSolver()->resetProblem();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Robot::insertRobotSRDFModelFromString(const char* robotName,
                                           const char* srdfString) {
  try {
    DevicePtr_t robot = getOrCreateRobot(problemSolver());
    srdf::loadModelFromXML(robot, robotName, srdfString);
    hpp::pinocchio::urdf::loadSRDFModelFromString(robot, robotName, srdfString);
    problemSolver()->resetProblem();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Robot::insertHumanoidModel(const char* robotName,
                                const char* rootJointType, const char* urdfName,
                                const char* srdfName) {
  try {
    DevicePtr_t robot = getOrCreateRobot(problemSolver());
    if (robot->robotFrames(robotName).size() > 0)
      HPP_THROW(std::invalid_argument,
                "A robot named " << robotName << " already exists");
    pinocchio::urdf::loadModel(robot, 0, robotName, rootJointType, urdfName,
                               srdfName);
    pinocchio::urdf::setupHumanoidRobot(robot, robotName);
    srdf::loadModelFromFile(robot, robotName, srdfName);
    problemSolver()->resetProblem();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Robot::insertHumanoidModelFromString(const char* robotName,
                                          const char* rootJointType,
                                          const char* urdfString,
                                          const char* srdfString) {
  try {
    DevicePtr_t robot = getOrCreateRobot(problemSolver());
    if (robot->robotFrames(robotName).size() > 0)
      HPP_THROW(std::invalid_argument,
                "A robot named " << robotName << " already exists");
    pinocchio::urdf::loadModelFromString(robot, 0, robotName, rootJointType,
                                         urdfString, srdfString);
    pinocchio::urdf::setupHumanoidRobot(robot, robotName);
    srdf::loadModelFromXML(robot, robotName, srdfString);
    problemSolver()->resetProblem();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Robot::loadEnvironmentModel(const char* urdfName, const char* srdfName,
                                 const char* prefix) {
  try {
    DevicePtr_t robot = getRobotOrThrow(problemSolver());

    std::string p(prefix);
    DevicePtr_t object = Device::create(p);
    pinocchio::urdf::loadModel(object, 0, "", "anchor", urdfName, "");
    srdf::loadModelFromFile(object, "", srdfName);
    object->controlComputation(hpp::pinocchio::JOINT_POSITION);
    object->computeForwardKinematics();
    object->updateGeometryPlacements();

    // Detach objects from joints
    problemSolver()->addObstacle(object, true, true);

    // Add contact shapes.
    typedef core::Container<JointAndShapes_t>::Map_t ShapeMap;
    const ShapeMap& m = object->jointAndShapes.map;
    for (ShapeMap::const_iterator it = m.begin(); it != m.end(); it++) {
      JointAndShapes_t shapes;
      for (JointAndShapes_t::const_iterator itT = it->second.begin();
           itT != it->second.end(); ++itT) {
        Transform3f M(Transform3f::Identity());
        if (itT->first) M = itT->first->currentTransformation();
        Shape_t newShape(itT->second.size());
        for (std::size_t i = 0; i < newShape.size(); ++i)
          newShape[i] = M.act(itT->second[i]);
        shapes.push_back(JointAndShape_t(JointPtr_t(), newShape));
      }
      problemSolver()->jointAndShapes.add(p + it->first, shapes);
    }

    copy(object->handles, robot->handles, robot, p);
    copy(object->grippers, robot->grippers, robot, p);
    problemSolver()->resetProblem();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Robot::loadEnvironmentModelFromString(const char* urdfString,
                                           const char* srdfString,
                                           const char* prefix) {
  try {
    DevicePtr_t robot = getRobotOrThrow(problemSolver());

    std::string p(prefix);
    DevicePtr_t object = Device::create(p);
    // TODO replace "" by p and remove `p +` in what follows
    pinocchio::urdf::loadModelFromString(object, 0, "", "anchor", urdfString,
                                         srdfString);
    srdf::loadModelFromXML(object, "", srdfString);
    object->controlComputation(hpp::pinocchio::JOINT_POSITION);
    object->computeForwardKinematics();
    object->updateGeometryPlacements();

    // Detach objects from joints
    problemSolver()->addObstacle(object, true, true);

    // Add contact shapes.
    typedef core::Container<JointAndShapes_t>::Map_t ShapeMap;
    const ShapeMap& m = object->jointAndShapes.map;
    for (ShapeMap::const_iterator it = m.begin(); it != m.end(); it++) {
      JointAndShapes_t shapes;
      for (JointAndShapes_t::const_iterator itT = it->second.begin();
           itT != it->second.end(); ++itT) {
        Transform3f M(Transform3f::Identity());
        if (itT->first) M = itT->first->currentTransformation();
        Shape_t newShape(itT->second.size());
        for (std::size_t i = 0; i < newShape.size(); ++i)
          newShape[i] = M.act(itT->second[i]);
        shapes.push_back(JointAndShape_t(JointPtr_t(), newShape));
      }
      problemSolver()->jointAndShapes.add(p + it->first, shapes);
    }

    copy(object->handles, robot->handles, robot, p);
    copy(object->grippers, robot->grippers, robot, p);
    problemSolver()->resetProblem();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

Transform__slice* Robot::getRootJointPosition(const char* robotName) {
  try {
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    std::string n(robotName);
    FrameIndices_t frameIdx = robot->robotFrames(robotName);
    if (frameIdx.size() == 0)
      throw hpp::Error("Root of subtree with the provided prefix not found");
    const pinocchio::Model& model = robot->model();
    const ::pinocchio::Frame& rf = model.frames[frameIdx[0]];
    double* res = new Transform_;
    if (rf.type == ::pinocchio::JOINT)
      Transform3fTohppTransform(model.jointPlacements[rf.parent], res);
    else
      Transform3fTohppTransform(rf.placement, res);
    return res;
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Robot::setRootJointPosition(const char* robotName,
                                 const ::hpp::Transform_ position) {
  try {
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    std::string n(robotName);
    Transform3f T;
    hppTransformToTransform3f(position, T);
    robot->setRobotRootPosition(n, T);
    robot->computeForwardKinematics();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Robot::addHandle(const char* linkName, const char* handleName,
                      const ::hpp::Transform_ localPosition, double clearance,
                      const ::hpp::boolSeq& mask) {
  try {
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    JointPtr_t joint = getJointByBodyNameOrThrow(problemSolver(), linkName);
    Transform3f T;

    const ::pinocchio::Frame& linkFrame =
        robot->model().frames[robot->model().getFrameId(std::string(linkName))];
    assert(linkFrame.type == ::pinocchio::BODY);

    pinocchio::JointIndex index(0);
    std::string jointName("universe");
    if (joint) {
      index = joint->index();
      jointName = joint->name();
    }
    hppTransformToTransform3f(localPosition, T);
    HandlePtr_t handle =
        Handle::create(handleName, linkFrame.placement * T, robot, joint);
    handle->clearance(clearance);
    handle->mask(corbaServer::boolSeqToVector(mask, 6));
    robot->handles.add(handleName, handle);
    assert(robot->model().existFrame(jointName));
    FrameIndex previousFrame(robot->model().getFrameId(jointName));
    robot->model().addFrame(::pinocchio::Frame(handleName, index, previousFrame,
                                               linkFrame.placement * T,
                                               ::pinocchio::OP_FRAME));
    // Recreate pinocchio data after modifying model
    robot->createData();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Robot::addGripper(const char* linkName, const char* gripperName,
                       const ::hpp::Transform_ p, double clearance) {
  try {
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    JointPtr_t joint = getJointByBodyNameOrThrow(problemSolver(), linkName);
    Transform3f T;

    const ::pinocchio::Frame& linkFrame =
        robot->model().frames[robot->model().getFrameId(std::string(linkName))];
    assert(linkFrame.type == ::pinocchio::BODY);

    pinocchio::JointIndex index(0);
    std::string jointName("universe");
    if (joint) {
      index = joint->index();
      jointName = joint->name();
    }
    hppTransformToTransform3f(p, T);
    assert(robot->model().existFrame(jointName));
    FrameIndex previousFrame(robot->model().getFrameId(jointName));
    robot->model().addFrame(
        ::pinocchio::Frame(gripperName, index, previousFrame,
                           linkFrame.placement * T, ::pinocchio::OP_FRAME));
    // Recreate pinocchio data after modifying model
    robot->createData();
    GripperPtr_t gripper = Gripper::create(gripperName, robot);
    gripper->clearance(clearance);
    robot->grippers.add(gripperName, gripper);
    // hppDout (info, "add Gripper: " << *gripper);
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

char* Robot::getGripperPositionInJoint(const char* gripperName,
                                       ::hpp::Transform__out position) {
  try {
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    GripperPtr_t gripper = robot->grippers.get(gripperName);
    if (!gripper) throw Error("This gripper does not exists.");
    const Transform3f& t = gripper->objectPositionInJoint();
    Transform3fTohppTransform(t, position);
    if (gripper->joint())
      return c_str(gripper->joint()->name());
    else
      return c_str("universe");
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

char* Robot::getHandlePositionInJoint(const char* handleName,
                                      ::hpp::Transform__out position) {
  try {
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    HandlePtr_t handle = robot->handles.get(handleName);
    if (!handle) throw Error("This handle does not exists.");
    const Transform3f& t = handle->localPosition();
    Transform3fTohppTransform(t, position);
    if (handle->joint())
      return c_str(handle->joint()->name());
    else
      return c_str("universe");
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

}  // namespace impl
}  // namespace manipulation
}  // namespace hpp
