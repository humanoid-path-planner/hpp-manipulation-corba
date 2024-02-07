// Copyright (c) 2014 CNRS
// Author: Florent Lamiraux
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

#ifndef HPP_MANIPULATION_CORBA_ROBOT_IMPL_HH
#define HPP_MANIPULATION_CORBA_ROBOT_IMPL_HH

#include <hpp/corbaserver/manipulation/fwd.hh>
#include <hpp/manipulation/problem-solver.hh>

#include "hpp/corbaserver/manipulation/robot-idl.hh"

namespace hpp {
namespace manipulation {
namespace impl {
using CORBA::Short;

class Robot : public virtual POA_hpp::corbaserver::manipulation::Robot {
 public:
  Robot();

  void setServer(Server* server) { server_ = server; }

  virtual void insertRobotModel(const char* robotName,
                                const char* rootJointType, const char* urdfName,
                                const char* srdfName);

  virtual void insertRobotModelOnFrame(const char* robotName,
                                       const char* frameName,
                                       const char* rootJointType,
                                       const char* urdfName,
                                       const char* srdfName);

  virtual void insertRobotModelFromString(const char* robotName,
                                          const char* rootJointType,
                                          const char* urdfString,
                                          const char* srdfString);

  virtual void insertRobotModelOnFrameFromString(const char* robotName,
                                                 const char* frameName,
                                                 const char* rootJointType,
                                                 const char* urdfString,
                                                 const char* srdfString);

  virtual void insertRobotSRDFModel(const char* robotName,
                                    const char* srdfPath);

  virtual void insertRobotSRDFModelFromString(const char* robotName,
                                              const char* srdfString);

  virtual void insertHumanoidModel(const char* robotName,
                                   const char* rootJointType,
                                   const char* urdfName, const char* srdfName);

  virtual void insertHumanoidModelFromString(const char* robotName,
                                             const char* rootJointType,
                                             const char* urdfString,
                                             const char* srdfString);

  virtual void loadEnvironmentModel(const char* urdfName, const char* srdfName,
                                    const char* prefix);

  virtual void loadEnvironmentModelFromString(const char* urdfString,
                                              const char* srdfString,
                                              const char* prefix);

  virtual Transform__slice* getRootJointPosition(const char* robotName);

  virtual void setRootJointPosition(const char* robotName,
                                    const ::hpp::Transform_ position);

  virtual void addHandle(const char* linkName, const char* handleName,
                         const ::hpp::Transform_ localPosition,
                         double clearance, const ::hpp::boolSeq& mask);

  virtual void addGripper(const char* linkName, const char* gripperName,
                          const ::hpp::Transform_ handlePositioninJoint,
                          double clearance);

  virtual char* getGripperPositionInJoint(const char* gripperName,
                                          ::hpp::Transform__out position);

  virtual char* getHandlePositionInJoint(const char* handleName,
                                         ::hpp::Transform__out position);

  virtual void setHandlePositionInJoint(const char* handleName,
                                        const ::hpp::Transform_ position);

 private:
  ProblemSolverPtr_t problemSolver();
  Server* server_;
};  // class Robot
}  // namespace impl
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_CORBA_ROBOT_IMPL_HH
