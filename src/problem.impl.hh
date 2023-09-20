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

#ifndef HPP_MANIPULATION_CORBA_PROBLEM_IMPL_HH
#define HPP_MANIPULATION_CORBA_PROBLEM_IMPL_HH

#include <hpp/corbaserver/manipulation/fwd.hh>
#include <hpp/manipulation/problem-solver.hh>

#include "hpp/corbaserver/manipulation/problem-idl.hh"
#include "hpp/manipulation_idl/_graph-idl.hh"
#include "hpp/manipulation_idl/_path_planners-idl.hh"

namespace hpp {
namespace manipulation {
namespace impl {
using CORBA::Double;
using CORBA::Short;
using CORBA::String_out;
using CORBA::ULong;
using CORBA::UShort;

class Problem : public virtual POA_hpp::corbaserver::manipulation::Problem {
 public:
  Problem();
  void setServer(Server* server) { server_ = server; }

  virtual bool selectProblem(const char* name);

  virtual void resetProblem();

  virtual Names_t* getAvailable(const char* what);

  virtual Names_t* getSelected(const char* what);

  virtual void loadRoadmap(const char* filename);

  virtual void createGrasp(const char* graspName, const char* gripperName,
                           const char* handleName);

  virtual void createPreGrasp(const char* graspName, const char* gripperName,
                              const char* handleName);

  virtual Names_t* getEnvironmentContactNames();

  virtual Names_t* getRobotContactNames();

  virtual Names_t* getEnvironmentContact(const char* name, intSeq_out indexes,
                                         floatSeqSeq_out points);

  virtual Names_t* getRobotContact(const char* name, intSeq_out indexes,
                                   floatSeqSeq_out points);

  virtual void createPlacementConstraint(const char* placName,
                                         const Names_t& shapeName,
                                         const Names_t& envContactName);

  virtual void createPrePlacementConstraint(const char* placName,
                                            const Names_t& shapeName,
                                            const Names_t& envContactName,
                                            CORBA::Double width);

  virtual void createQPStabilityConstraint(const char* constraintName,
                                           const char* comRootJointName,
                                           const Names_t& shapesName);

  virtual bool setConstraints(hpp::ID id, bool target);

  virtual void registerConstraints(const char* constraint,
                                   const char* complement, const char* both);

  virtual bool applyConstraints(hpp::ID id, const hpp::floatSeq& input,
                                hpp::floatSeq_out output,
                                double& residualError);

  virtual bool applyConstraintsWithOffset(hpp::ID IDedge,
                                          const hpp::floatSeq& qnear,
                                          const hpp::floatSeq& input,
                                          hpp::floatSeq_out output,
                                          double& residualError);

  virtual bool buildAndProjectPath(hpp::ID IDedge, const hpp::floatSeq& qb,
                                   const hpp::floatSeq& qe,
                                   CORBA::Long& indexNotProj,
                                   CORBA::Long& indexProj);

  virtual void setTargetState(hpp::ID IDstate);

  virtual ID edgeAtParam(ULong pathId, Double param, String_out name);

  hpp::manipulation_idl::graph_idl::Validation_ptr createGraphValidation();

  core_idl::Roadmap_ptr readRoadmap(
      const char* filename, pinocchio_idl::Device_ptr robot,
      manipulation_idl::graph_idl::Graph_ptr graph);
  void writeRoadmap(const char* filename, core_idl::Roadmap_ptr roadmap,
                    pinocchio_idl::Device_ptr robot,
                    manipulation_idl::graph_idl::Graph_ptr graph);

  core_idl::Roadmap_ptr createRoadmap(core_idl::Distance_ptr distance,
                                      pinocchio_idl::Device_ptr robot);
  virtual hpp::core_idl::PathPlanner_ptr createTransitionPlanner();

 private:
  ProblemSolverPtr_t problemSolver();
  graph::GraphPtr_t graph(bool throwIfNull = true);
  Server* server_;
};  // class Problem
}  // namespace impl
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_CORBA_PROBLEM_IMPL_HH
