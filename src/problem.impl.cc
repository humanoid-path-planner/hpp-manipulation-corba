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

#include "problem.impl.hh"

#include <hpp/constraints/convex-shape-contact.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/corbaserver/conversions.hh>
#include <hpp/corbaserver/manipulation/server.hh>
#include <hpp/corbaserver/servant-base.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/parser/roadmap.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/manipulation/path-planner/transition-planner.hh>
#include <hpp/manipulation/roadmap.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/serialization.hh>
#include <hpp/util/debug.hh>
#ifdef HPP_CONSTRAINTS_USE_QPOASES
#include <hpp/constraints/qp-static-stability.hh>
#endif
#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/state.hh>
#include <hpp/manipulation/graph/validation.hh>
#include <hpp/manipulation/manipulation-planner.hh>
#include <hpp/manipulation/problem.hh>
#include <hpp/manipulation/roadmap.hh>
#include <hpp/manipulation/serialization.hh>
#include <hpp/manipulation/steering-method/graph.hh>

#include "hpp/core_idl/distances-fwd.hh"      // For hpp::core_impl::Roadmap
#include "hpp/core_idl/path_planners-fwd.hh"  // For hpp::core_impl::Roadmap
#include "hpp/manipulation_idl/_graph.hh"
#include "hpp/manipulation_idl/_path_planners.hh"
#include "hpp/manipulation_idl/device-fwd.hh"
#include "hpp/pinocchio_idl/robots-fwd.hh"
#include "tools.hh"

namespace hpp {
namespace manipulation {
namespace impl {
using corbaServer::makeServantDownCast;
using corbaServer::reference_to_object;

namespace {
using corbaServer::floatSeqToConfigPtr;
typedef core::ProblemSolver CPs_t;

Names_t* jointAndShapes(const JointAndShapes_t& js, intSeq_out indexes_out,
                        floatSeqSeq_out points) {
  char** nameList = Names_t::allocbuf((ULong)js.size());
  Names_t* jointNames =
      new Names_t((ULong)js.size(), (ULong)js.size(), nameList);

  intSeq* indexes = new intSeq();
  indexes->length((ULong)js.size());
  indexes_out = indexes;

  CORBA::Long rank = 0;
  std::size_t nbPts = 0;
  for (JointAndShapes_t::const_iterator itJs = js.begin(); itJs != js.end();
       ++itJs) {
    if (itJs->first) {
      nameList[rank] = new char[itJs->first->name().length() + 1];
      strcpy(nameList[rank], itJs->first->name().c_str());
    } else {
      nameList[rank] = new char[9];
      strcpy(nameList[rank], "universe");
    }
    nbPts += itJs->second.size();
    (*indexes)[rank] = (int)nbPts;
    ++rank;
  }

  floatSeqSeq* pts = new hpp::floatSeqSeq();
  points = pts;
  pts->length((CORBA::ULong)nbPts);

  rank = 0;
  ULong iJs = 0;
  for (JointAndShapes_t::const_iterator itJs = js.begin(); itJs != js.end();
       ++itJs) {
    for (std::size_t i = 0; i < itJs->second.size(); ++i) {
      floatSeq p;
      p.length(3);
      p[0] = itJs->second[i][0];
      p[1] = itJs->second[i][1];
      p[2] = itJs->second[i][2];
      (*pts)[rank] = p;
      ++rank;
    }
    assert((*indexes)[iJs] == rank);
    ++iJs;
  }
  return jointNames;
}
}  // namespace

Problem::Problem() : server_(0x0) {}

ProblemSolverPtr_t Problem::problemSolver() { return server_->problemSolver(); }

graph::GraphPtr_t Problem::graph(bool throwIfNull) {
  graph::GraphPtr_t g = problemSolver()->constraintGraph();
  if (throwIfNull && !g) throw Error("You should create the graph");
  return g;
}

bool Problem::selectProblem(const char* name) {
  std::string psName(name);
  corbaServer::ProblemSolverMapPtr_t psMap(server_->problemSolverMap());
  bool has = psMap->has(psName);
  if (!has) psMap->add(psName, ProblemSolver::create());
  psMap->selected(psName);
  return !has;
}

void Problem::resetProblem() {
  corbaServer::ProblemSolverMapPtr_t psMap(server_->problemSolverMap());
  psMap->replaceSelected(ProblemSolver::create());
}

Names_t* Problem::getAvailable(const char* what) {
  std::string w(what);
  std::transform(w.begin(), w.end(), w.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  typedef std::list<std::string> Ret_t;
  Ret_t ret;

  if (w == "gripper") {
    ret = getRobotOrThrow(problemSolver())->grippers.getKeys<Ret_t>();
  } else if (w == "handle") {
    ret = getRobotOrThrow(problemSolver())->handles.getKeys<Ret_t>();
  } else if (w == "robotcontact") {
    ret = getRobotOrThrow(problemSolver())->jointAndShapes.getKeys<Ret_t>();
  } else if (w == "envcontact") {
    ret = problemSolver()->jointAndShapes.getKeys<Ret_t>();
  } else if (w == "constraintgraph") {
    ret = problemSolver()->graphs.getKeys<Ret_t>();
  } else if (w == "type") {
    ret = {"Gripper", "Handle", "RobotContact", "EnvContact",
           "ConstraintGraph"};
  } else {
    throw Error(("Type \"" + std::string(what) + "\" not known").c_str());
  }

  return toNames_t(ret.begin(), ret.end());
}

Names_t* Problem::getSelected(const char* what) {
  std::string w(what);
  std::transform(w.begin(), w.end(), w.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  typedef std::list<std::string> Ret_t;
  Ret_t ret;

  if (w == "constraintgraph") {
    if (problemSolver()->constraintGraph())
      ret.push_back(problemSolver()->constraintGraph()->name());
    else
      throw Error("No constraint graph selected.");
  } else if (w == "type") {
    ret = {"ConstraintGraph"};
  } else {
    throw Error(("Type \"" + std::string(what) + "\" not known").c_str());
  }

  return toNames_t(ret.begin(), ret.end());
}

void Problem::loadRoadmap(const char* filename) {
  try {
    ProblemSolverPtr_t ps(problemSolver());
    DevicePtr_t robot = getRobotOrThrow(ps);
    graph::GraphPtr_t g = ps->constraintGraph();

    using namespace core::parser;

    typedef hpp::serialization::archive_tpl<
        boost::archive::binary_iarchive,
        hpp::serialization::remove_duplicate::vector_archive>
        archive_type;
    hpp::core::RoadmapPtr_t roadmap;
    serializeRoadmap<archive_type>(roadmap, std::string(filename),
                                   make_nvp(robot->name(), robot.get()),
                                   make_nvp(g->name(), g.get()));
    problemSolver()->roadmap(roadmap);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Problem::createGrasp(const char* graspName, const char* gripperName,
                          const char* handleName) {
  try {
    problemSolver()->createGraspConstraint(graspName, gripperName, handleName);
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Problem::createPreGrasp(const char* graspName, const char* gripperName,
                             const char* handleName) {
  try {
    problemSolver()->createPreGraspConstraint(graspName, gripperName,
                                              handleName);
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

Names_t* Problem::getEnvironmentContactNames() {
  try {
    typedef std::map<std::string, JointAndShapes_t> ShapeMap;
    const ShapeMap& m = problemSolver()->jointAndShapes.map;

    char** nameList = Names_t::allocbuf((ULong)m.size());
    Names_t* jointNames =
        new Names_t((ULong)m.size(), (ULong)m.size(), nameList);

    std::size_t rank = 0;
    for (ShapeMap::const_iterator it = m.begin(); it != m.end(); it++) {
      nameList[rank] = (char*)malloc(sizeof(char) * (it->first.length() + 1));
      strcpy(nameList[rank], it->first.c_str());
      rank++;
    }
    return jointNames;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

Names_t* Problem::getRobotContactNames() {
  try {
    typedef std::map<std::string, JointAndShapes_t> ShapeMap;
    DevicePtr_t r = getRobotOrThrow(problemSolver());
    const ShapeMap& m = r->jointAndShapes.map;

    char** nameList = Names_t::allocbuf((ULong)m.size());
    Names_t* jointNames =
        new Names_t((ULong)m.size(), (ULong)m.size(), nameList);

    std::size_t rank = 0;
    for (ShapeMap::const_iterator it = m.begin(); it != m.end(); it++) {
      nameList[rank] = (char*)malloc(sizeof(char) * (it->first.length() + 1));
      strcpy(nameList[rank], it->first.c_str());
      rank++;
    }
    return jointNames;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

Names_t* Problem::getEnvironmentContact(const char* name, intSeq_out indexes,
                                        floatSeqSeq_out points) {
  try {
    const JointAndShapes_t& js = problemSolver()->jointAndShapes.get(name);

    return jointAndShapes(js, indexes, points);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

Names_t* Problem::getRobotContact(const char* name, intSeq_out indexes,
                                  hpp::floatSeqSeq_out points) {
  try {
    DevicePtr_t r = getRobotOrThrow(problemSolver());
    const JointAndShapes_t& js = r->jointAndShapes.get(name);

    return jointAndShapes(js, indexes, points);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Problem::createPlacementConstraint(const char* placName,
                                        const Names_t& surface1,
                                        const Names_t& surface2) {
  try {
    problemSolver()->createPlacementConstraint(
        placName, corbaServer::toStrings<std::vector<std::string> >(surface1),
        corbaServer::toStrings<std::vector<std::string> >(surface2), 1e-3);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Problem::createPrePlacementConstraint(const char* placName,
                                           const Names_t& surface1,
                                           const Names_t& surface2,
                                           CORBA::Double width) {
  try {
    problemSolver()->createPrePlacementConstraint(
        placName, corbaServer::toStrings<std::vector<std::string> >(surface1),
        corbaServer::toStrings<std::vector<std::string> >(surface2), width,
        1e-3);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Problem::createQPStabilityConstraint(const char* constraintName,
                                          const char* comRootJointName,
                                          const Names_t& shapesName) {
  try {
#ifdef HPP_CONSTRAINTS_USE_QPOASES
    // Get robot in hppPlanner object.
    DevicePtr_t robot = getRobotOrThrow(problemSolver());

    using constraints::ConvexShape;
    using constraints::QPStaticStability;
    using constraints::QPStaticStabilityPtr_t;
    typedef constraints::QPStaticStability::ForceData ForceData;
    using pinocchio::CenterOfMassComputation;
    using pinocchio::CenterOfMassComputationPtr_t;

    std::vector<ForceData> fds;
    std::size_t nbPoints = 0;

    for (CORBA::ULong i = 0; i < shapesName.length(); ++i) {
      JointAndShapes_t l =
          robot->jointAndShapes.get(std::string(shapesName[i]));
      if (l.empty()) throw Error("Robot shapes not found.");
      for (JointAndShapes_t::const_iterator it = l.begin(); it != l.end();
           it++) {
        ConvexShape c(ConvexShape(it->second, it->first));
        ForceData fd;
        fd.joint = c.joint_;
        fd.supportJoint = JointPtr_t();
        fd.normal = -c.N_;
        fd.points = c.Pts_;
        nbPoints += c.Pts_.size();
        fds.push_back(fd);
      }
    }

    CenterOfMassComputationPtr_t com = CenterOfMassComputation::create(robot);
    JointPtr_t comRJ(robot->getJointByName(comRootJointName));
    com->add(comRJ);
    QPStaticStabilityPtr_t c =
        QPStaticStability::create(constraintName, robot, fds, com);
    problemSolver()->addNumericalConstraint(
        constraintName, Implicit::create(c, constraints::ComparisonTypes_t(
                                                1, constraints::EqualToZero)));
#else
    // Avoid unused-variable compilation warnings
    (void)constraintName;
    (void)shapesName;
    throw std::runtime_error(
        "Problem::createQPStabilityConstraint is not implemented");
#endif
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

bool Problem::setConstraints(hpp::ID id, bool target) {
  /// First get the constraint.
  ConstraintSetPtr_t constraint;
  try {
    graph::GraphComponentPtr_t comp = graph()->get((size_t)id).lock();
    graph::EdgePtr_t edge = HPP_DYNAMIC_PTR_CAST(graph::Edge, comp);
    graph::StatePtr_t state = HPP_DYNAMIC_PTR_CAST(graph::State, comp);
    if (edge) {
      if (target)
        constraint = graph()->targetConstraint(edge);
      else
        constraint = graph()->pathConstraint(edge);
    } else if (state) {
      constraint = graph()->configConstraint(state);
    } else {
      std::stringstream ss;
      ss << "ID " << id << " is neither an edge nor a state";
      std::string errmsg = ss.str();
      throw Error(errmsg.c_str());
    }
    if (constraint) {
      problemSolver()->resetConstraints();
      problemSolver()->addConstraint(constraint->copy());
      return true;
    } else
      return false;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Problem::registerConstraints(const char* constraint,
                                  const char* complement, const char* both) {
  using constraints::ImplicitPtr_t;
  try {
    ImplicitPtr_t constr(problemSolver()->numericalConstraints.get(constraint));
    ImplicitPtr_t comp(problemSolver()->numericalConstraints.get(complement));
    ImplicitPtr_t comb(problemSolver()->numericalConstraints.get(both));
    problemSolver()->constraintsAndComplements.push_back(
        ConstraintAndComplement_t(constr, comp, comb));
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

bool Problem::applyConstraints(hpp::ID id, const hpp::floatSeq& input,
                               hpp::floatSeq_out output,
                               double& residualError) {
  /// First get the constraint.
  ConstraintSetPtr_t constraint;
  try {
    graph::GraphComponentPtr_t comp = graph()->get((size_t)id).lock();
    graph::EdgePtr_t edge = HPP_DYNAMIC_PTR_CAST(graph::Edge, comp);
    graph::StatePtr_t state = HPP_DYNAMIC_PTR_CAST(graph::State, comp);
    if (edge) {
      constraint = graph(false)->targetConstraint(edge);
      DevicePtr_t robot = getRobotOrThrow(problemSolver());
      if (core::ConfigProjectorPtr_t cp = constraint->configProjector()) {
        cp->rightHandSideFromConfig(robot->currentConfiguration());
      }
    } else if (state)
      constraint = graph(false)->configConstraint(state);
    else {
      std::stringstream ss;
      ss << "ID " << id << " is neither an edge nor a state";
      std::string errmsg = ss.str();
      throw Error(errmsg.c_str());
    }
    bool success = false;
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    ConfigurationPtr_t config = floatSeqToConfigPtr(robot, input, true);
    success = constraint->apply(*config);
    if (hpp::core::ConfigProjectorPtr_t configProjector =
            constraint->configProjector()) {
      residualError = configProjector->residualError();
    }
    output = vectorToFloatSeq(*config);
    return success;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

bool Problem::applyConstraintsWithOffset(hpp::ID IDedge,
                                         const hpp::floatSeq& qnear,
                                         const hpp::floatSeq& input,
                                         hpp::floatSeq_out output,
                                         double& residualError) {
  /// First get the constraint.
  graph::EdgePtr_t edge;
  try {
    edge =
        HPP_DYNAMIC_PTR_CAST(graph::Edge, graph()->get((size_t)IDedge).lock());
    if (!edge) {
      std::stringstream ss;
      ss << "ID " << IDedge << " is not an edge";
      std::string errmsg = ss.str();
      throw Error(errmsg.c_str());
    }
    bool success = false;
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    ConfigurationPtr_t config = floatSeqToConfigPtr(robot, input, true);
    ConfigurationPtr_t qoffset = floatSeqToConfigPtr(robot, qnear, true);
    value_type dist = 0;
    core::NodePtr_t nNode =
        problemSolver()->roadmap()->nearestNode(qoffset, dist);
    if (dist < 1e-8)
      success = edge->generateTargetConfig(nNode, *config);
    else
      success = edge->generateTargetConfig(*qoffset, *config);

    hpp::core::ConfigProjectorPtr_t configProjector(
        edge->targetConstraint()->configProjector());
    if (configProjector) {
      residualError = configProjector->residualError();
    } else {
      hppDout(info, "No config projector.");
    }
    ULong size = (ULong)config->size();
    hpp::floatSeq* q_ptr = new hpp::floatSeq();
    q_ptr->length(size);

    for (std::size_t i = 0; i < size; ++i) {
      (*q_ptr)[(ULong)i] = (*config)[i];
    }
    output = q_ptr;
    return success;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

bool Problem::buildAndProjectPath(hpp::ID IDedge, const hpp::floatSeq& qb,
                                  const hpp::floatSeq& qe,
                                  CORBA::Long& indexNotProj,
                                  CORBA::Long& indexProj) {
  /// First get the constraint.
  graph::EdgePtr_t edge;
  try {
    edge =
        HPP_DYNAMIC_PTR_CAST(graph::Edge, graph()->get((size_t)IDedge).lock());
    if (!edge) {
      std::stringstream ss;
      ss << "ID " << IDedge << " is not an edge";
      std::string errmsg = ss.str();
      throw Error(errmsg.c_str());
    }
    // If steering method is not completely set in the graph, create
    // one.
    if (!edge->parentGraph()->problem()->manipulationSteeringMethod() ||
        !edge->parentGraph()
             ->problem()
             ->manipulationSteeringMethod()
             ->innerSteeringMethod()) {
      problemSolver()->initSteeringMethod();
      if (!edge->parentGraph()->problem()->manipulationSteeringMethod() ||
          !edge->parentGraph()
               ->problem()
               ->manipulationSteeringMethod()
               ->innerSteeringMethod())
        throw Error("Could not initialize the steering method.");
    }
    bool success = false;
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    ConfigurationPtr_t q1 = floatSeqToConfigPtr(robot, qb, true);
    ConfigurationPtr_t q2 = floatSeqToConfigPtr(robot, qe, true);
    core::PathVectorPtr_t pv;
    indexNotProj = -1;
    indexProj = -1;
    core::PathPtr_t path;
    success = edge->build(path, *q1, *q2);
    if (!success) return false;
    pv = HPP_DYNAMIC_PTR_CAST(core::PathVector, path);
    indexNotProj = (CORBA::Long)problemSolver()->paths().size();
    if (!pv) {
      pv = core::PathVector::create(path->outputSize(),
                                    path->outputDerivativeSize());
      pv->appendPath(path);
    }
    problemSolver()->addPath(pv);

    core::PathPtr_t projPath;
    PathProjectorPtr_t pathProjector(
        problemSolver()->problem()->pathProjector());
    if (!pathProjector) {
      problemSolver()->initPathProjector();
      pathProjector = problemSolver()->problem()->pathProjector();
    }
    if (pathProjector) {
      success = pathProjector->apply(path, projPath);
    } else {
      success = true;
      projPath = path->copy();
    }

    if (!success) {
      if (!projPath || projPath->length() == 0) return false;
    }
    pv = HPP_DYNAMIC_PTR_CAST(core::PathVector, projPath);
    indexProj = (CORBA::Long)problemSolver()->paths().size();
    if (!pv) {
      pv = core::PathVector::create(projPath->outputSize(),
                                    projPath->outputDerivativeSize());
      pv->appendPath(projPath);
    }
    problemSolver()->addPath(pv);
    return success;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Problem::setTargetState(hpp::ID IDstate) {
  try {
    graph::GraphComponentPtr_t comp = graph()->get((size_t)IDstate).lock();
    graph::StatePtr_t state = HPP_DYNAMIC_PTR_CAST(graph::State, comp);
    if (!state) {
      HPP_THROW(Error, "ID " << IDstate << " is not a state.");
    }
    problemSolver()->setTargetState(state);
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

ID Problem::edgeAtParam(ULong pathId, Double param, String_out name) {
  try {
    if (pathId >= problemSolver()->paths().size()) {
      HPP_THROW(Error, "Wrong path id: " << pathId << ", number path: "
                                         << problemSolver()->paths().size()
                                         << ".");
    }
    core::PathVectorPtr_t path = problemSolver()->paths()[pathId];
    core::PathVectorPtr_t flat = core::PathVector::create(
        path->outputSize(), path->outputDerivativeSize());
    path->flatten(flat);
    value_type unused;
    std::size_t r = flat->rankAtParam(param, unused);
    core::PathPtr_t p = flat->pathAtRank(r);
    manipulation::ConstraintSetPtr_t constraint =
        HPP_DYNAMIC_PTR_CAST(manipulation::ConstraintSet, p->constraints());
    if (!constraint) {
      HPP_THROW(Error, "Path constraint is not of the good type "
                           << "at id " << pathId << ", param " << param
                           << " (rank: " << r << ")");
    }
    if (!constraint->edge()) {
      HPP_THROW(Error, "Path constraint does not contain edge information "
                           << "at id " << pathId << ", param " << param
                           << " (rank: " << r << ")");
    }
    if (constraint->edge()->parentGraph())
      name = constraint->edge()->parentGraph()->name().c_str();
    else
      name = "Parent graph was destroyed.";
    return (ID)constraint->edge()->id();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

manipulation_idl::graph_idl::Validation_ptr Problem::createGraphValidation() {
  core::ProblemSolverPtr_t ps = problemSolver();
  graph::ValidationPtr_t validation(new graph::Validation(ps->problem()));

  typedef manipulation_impl::graph_impl::Validation Validation_impl;

  manipulation_idl::graph_idl::Validation_var validation_idl =
      makeServantDownCast<Validation_impl>(
          server_->parent(), Validation_impl::Storage(validation));
  return validation_idl._retn();
}

core_idl::Roadmap_ptr Problem::readRoadmap(
    const char* filename, pinocchio_idl::Device_ptr robot,
    manipulation_idl::graph_idl::Graph_ptr graph) {
  try {
    pinocchio::DevicePtr_t device =
        reference_to_object<pinocchio::Device>(server_->parent(), robot);
    graph::GraphPtr_t _graph =
        reference_to_object<graph::Graph>(server_->parent(), graph);

    hpp::core::RoadmapPtr_t roadmap;
    std::string fn(filename);
    bool xml = (fn.size() >= 4 && fn.compare(fn.size() - 4, 4, ".xml") == 0);
    using namespace core::parser;
    if (xml) {
      typedef hpp::serialization::archive_tpl<
          boost::archive::xml_iarchive,
          hpp::serialization::remove_duplicate::vector_archive>
          archive_type;
      serializeRoadmap<archive_type>(roadmap, fn,
                                     make_nvp(device->name(), device.get()),
                                     make_nvp(_graph->name(), _graph.get()));
    } else {
      typedef hpp::serialization::archive_tpl<
          boost::archive::binary_iarchive,
          hpp::serialization::remove_duplicate::vector_archive>
          archive_type;
      serializeRoadmap<archive_type>(roadmap, fn,
                                     make_nvp(device->name(), device.get()),
                                     make_nvp(_graph->name(), _graph.get()));
    }

    core_idl::Roadmap_var o =
        makeServantDownCast<core_impl::Roadmap>(server_->parent(), roadmap);
    return o._retn();
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Problem::writeRoadmap(const char* filename, core_idl::Roadmap_ptr _roadmap,
                           pinocchio_idl::Device_ptr robot,
                           manipulation_idl::graph_idl::Graph_ptr graph) {
  try {
    pinocchio::DevicePtr_t device =
        reference_to_object<pinocchio::Device>(server_->parent(), robot);
    graph::GraphPtr_t _graph =
        reference_to_object<graph::Graph>(server_->parent(), graph);
    core::RoadmapPtr_t roadmap =
        reference_to_object<core::Roadmap>(server_->parent(), _roadmap);

    std::string fn(filename);
    bool xml = (fn.size() >= 4 && fn.compare(fn.size() - 4, 4, ".xml") == 0);
    using namespace core::parser;
    if (xml) {
      typedef hpp::serialization::archive_tpl<
          boost::archive::xml_oarchive,
          hpp::serialization::remove_duplicate::vector_archive>
          archive_type;
      serializeRoadmap<archive_type>(roadmap, fn,
                                     make_nvp(device->name(), device.get()),
                                     make_nvp(_graph->name(), _graph.get()));
    } else {
      typedef hpp::serialization::archive_tpl<
          boost::archive::binary_oarchive,
          hpp::serialization::remove_duplicate::vector_archive>
          archive_type;
      serializeRoadmap<archive_type>(roadmap, fn,
                                     make_nvp(device->name(), device.get()),
                                     make_nvp(_graph->name(), _graph.get()));
    }
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

core_idl::Roadmap_ptr Problem::createRoadmap(core_idl::Distance_ptr distance,
                                             pinocchio_idl::Device_ptr robot) {
  core_idl::Roadmap_var o = makeServantDownCast<core_impl::Roadmap>(
      server_->parent(),
      Roadmap::create(
          reference_to_object<core::Distance>(server_->parent(), distance),
          reference_to_object<pinocchio::Device>(server_->parent(), robot)));
  return o._retn();
}
core_idl::PathPlanner_ptr Problem::createTransitionPlanner() {
  ProblemSolverPtr_t ps = problemSolver();
  core::DistancePtr_t dist(core::WeighedDistance::create(ps->robot()));
  core::RoadmapPtr_t roadmap(core::Roadmap::create(dist, ps->robot()));
  ProblemPtr_t problem(ps->problem());
  core::PathPlannerPtr_t planner(
      manipulation::pathPlanner::TransitionPlanner::createWithRoadmap(problem,
                                                                      roadmap));
  core_idl::PathPlanner_var o =
      makeServantDownCast<core_impl::PathPlanner>(server_->parent(), planner);
  return o._retn();
}
}  // namespace impl
}  // namespace manipulation
}  // namespace hpp
