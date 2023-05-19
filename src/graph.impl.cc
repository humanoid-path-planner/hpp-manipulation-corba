// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#include "graph.impl.hh"

#include <fstream>
#include <hpp/constraints/differentiable-function.hh>
#include <hpp/corbaserver/conversions.hh>
#include <hpp/corbaserver/manipulation/server.hh>
#include <hpp/manipulation/connected-component.hh>
#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/graph/guided-state-selector.hh>
#include <hpp/manipulation/graph/helper.hh>
#include <hpp/manipulation/graph/state-selector.hh>
#include <hpp/manipulation/graph/state.hh>
#include <hpp/manipulation/manipulation-planner.hh>
#include <hpp/manipulation/problem.hh>
#include <hpp/manipulation/roadmap.hh>
#include <hpp/manipulation/steering-method/graph.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>
#include <hpp/util/pointer.hh>
#include <pinocchio/multibody/model.hpp>
#include <sstream>

#include "tools.hh"

namespace hpp {
namespace manipulation {
using constraints::Implicit;
namespace impl {
using CORBA::ULong;
using corbaServer::floatSeqToConfig;
using corbaServer::floatSeqToConfigPtr;
using graph::Edge;
using graph::EdgePtr_t;
using graph::LevelSetEdge;
using graph::LevelSetEdgePtr_t;
using graph::WaypointEdge;
using graph::WaypointEdgePtr_t;

namespace {
typedef core::ProblemSolver CPs_t;

template <typename T>
std::string toStr() {
  return typeid(T).name();
}
template <>
std::string toStr<graph::State>() {
  return "Node";
}
template <>
std::string toStr<graph::Edge>() {
  return "Edge";
}
template <>
std::string toStr<graph::Graph>() {
  return "Graph";
}
template <>
std::string toStr<graph::LevelSetEdge>() {
  return "LevelSetEdge";
}
template <>
std::string toStr<graph::WaypointEdge>() {
  return "WaypointEdge";
}

std::vector<graph::helper::ObjectDef_t> toObjectVector(const Names_t& names,
                                                       const Namess_t& hsPO,
                                                       const Namess_t& shPO) {
  using graph::helper::ObjectDef_t;
  std::vector<graph::helper::ObjectDef_t> ret;
  // Check size of lists
  if (hsPO.length() != names.length()) {
    HPP_THROW(Error, "Number of handle lists ("
                         << hsPO.length()
                         << ") does not match number of objects ("
                         << names.length() << ").");
  }
  if (shPO.length() != names.length()) {
    HPP_THROW(Error, "Number of contact lists ("
                         << shPO.length()
                         << ") does not match number of objects ("
                         << names.length() << ").");
  }
  for (ULong i = 0; i < names.length(); ++i) {
    ret.push_back(ObjectDef_t());
    ObjectDef_t& od = ret.back();
    od.name = names[i];
    od.handles = corbaServer::toStrings<std::vector<std::string> >(hsPO[i]);
    od.shapes = corbaServer::toStrings<std::vector<std::string> >(shPO[i]);
  }
  return ret;
}

void setRule(const hpp::corbaserver::manipulation::Rule& in,
             graph::helper::Rule& out) {
  out.grippers_ = toStringVector(in.grippers);
  out.handles_ = toStringVector(in.handles);
  out.link_ = in.link;
}
}  // namespace

Graph::Graph() : server_(0x0) {}

ProblemSolverPtr_t Graph::problemSolver() { return server_->problemSolver(); }

graph::GraphPtr_t Graph::graph(bool throwIfNull) {
  graph::GraphPtr_t g = problemSolver()->constraintGraph();
  if (throwIfNull && !g) throw Error("You should create the graph");
  return g;
}

template <typename T>
shared_ptr<T> Graph::getComp(ID id, bool throwIfWrongType) {
  shared_ptr<T> comp;
  try {
    comp = HPP_DYNAMIC_PTR_CAST(T, graph()->get(id).lock());
  } catch (std::out_of_range& e) {
    throw Error(e.what());
  }
  if (throwIfWrongType && !comp) {
    std::stringstream ss;
    ss << "ID " << id << " is not a " << toStr<T>();
    throw Error(ss.str().c_str());
  }
  return comp;
}

Long Graph::createGraph(const char* graphName) {
  try {
    std::string name(graphName);
    if (problemSolver()->graphs.has(name)) {
      HPP_THROW(Error, "A graph named " << name << " already exists");
    }

    DevicePtr_t robot = problemSolver()->robot();
    if (!robot) throw Error("Build the robot first.");
    // Create default steering method to store in edges, until we define a
    // factory for steering methods.
    graph::GraphPtr_t g =
        graph::Graph::create(name, robot, problemSolver()->problem());
    g->maxIterations(problemSolver()->maxIterProjection());
    g->errorThreshold(problemSolver()->errorThreshold());

    problemSolver()->graphs.add(name, g);
    problemSolver()->constraintGraph(name);
    return (Long)g->id();
  } catch (std::exception& e) {
    throw Error(e.what());
  }
}

void Graph::deleteGraph(const char* graphName) {
  try {
    std::string name(graphName);
    if (!problemSolver()->graphs.has(name)) {
      HPP_THROW(Error, "There is no graph named " << name << ".");
    }
    problemSolver()->graphs.erase(name);
  } catch (std::exception& e) {
    throw Error(e.what());
  }
}

void Graph::selectGraph(const char* graphName) {
  try {
    problemSolver()->constraintGraph(graphName);
  } catch (const std::exception& e) {
    throw Error(e.what());
  }
}

void Graph::createSubGraph(const char* subgraphName) {
  try {
    graph::GuidedStateSelectorPtr_t ns = graph::GuidedStateSelector::create(
        subgraphName, problemSolver()->roadmap());
    graph()->stateSelector(ns);
  } catch (const std::exception& exc) {
    throw(Error(exc.what()));
  }
}

void Graph::setTargetNodeList(const ID graphId, const hpp::IDseq& nodes) {
  graph::GraphPtr_t graph = getComp<graph::Graph>(graphId);
  graph::GuidedStateSelectorPtr_t ns =
      HPP_DYNAMIC_PTR_CAST(graph::GuidedStateSelector, graph->stateSelector());
  if (!ns)
    throw Error("The state selector is not of type GuidedStateSelector.");
  try {
    graph::States_t nl;
    for (unsigned int i = 0; i < nodes.length(); ++i)
      nl.push_back(getComp<graph::State>(nodes[i]));
    ns->setStateList(nl);
  } catch (std::out_of_range& e) {
    throw Error(e.what());
  }
}

Long Graph::createNode(const Long graphId, const char* nodeName,
                       const bool waypoint, const Long priority) {
  graph::GraphPtr_t graph = getComp<graph::Graph>(graphId);
  if (graph->stateSelector()) {
    graph::StatePtr_t state =
        graph->stateSelector()->createState(nodeName, waypoint, priority);
    return (Long)state->id();
  } else {
    throw Error("Graph has no state selector.");
  }
}

Long Graph::createEdge(const Long nodeFromId, const Long nodeToId,
                       const char* edgeName, const Long w,
                       const Long isInNodeId) {
  graph::StatePtr_t from = getComp<graph::State>(nodeFromId),
                    to = getComp<graph::State>(nodeToId),
                    isInState = getComp<graph::State>(isInNodeId);

  EdgePtr_t edge = from->linkTo(edgeName, to, (size_type)w,
                                (graph::State::EdgeFactory)Edge::create);
  edge->state(isInState);

  return (Long)edge->id();
}

Long Graph::createWaypointEdge(const Long nodeFromId, const Long nodeToId,
                               const char* edgeName, const Long nb,
                               const Long w, const Long isInNodeId) {
  try {
    graph::StatePtr_t from = getComp<graph::State>(nodeFromId),
                      to = getComp<graph::State>(nodeToId),
                      isInNode = getComp<graph::State>(isInNodeId);

    EdgePtr_t edge_pc =
        from->linkTo(edgeName, to, (size_type)w,
                     (graph::State::EdgeFactory)WaypointEdge::create);

    edge_pc->state(isInNode);
    WaypointEdgePtr_t edge = HPP_DYNAMIC_PTR_CAST(WaypointEdge, edge_pc);

    edge->nbWaypoints(nb);
    return (Long)edge->id();
  } catch (const std::exception& exc) {
    throw(Error(exc.what()));
  }
}

void Graph::setWaypoint(const ID waypointEdgeId, const Long index,
                        const ID edgeId, const ID nodeId) {
  try {
    WaypointEdgePtr_t we = getComp<graph::WaypointEdge>(waypointEdgeId);
    EdgePtr_t edge = getComp<Edge>(edgeId);
    graph::StatePtr_t state = getComp<graph::State>(nodeId);

    if (index < 0 || (std::size_t)index > we->nbWaypoints())
      throw Error("Invalid index");
    we->setWaypoint(index, edge, state);
  } catch (const std::exception& exc) {
    throw(Error(exc.what()));
  }
}

void Graph::getGraph(GraphComp_out graph_out, GraphElements_out elmts) {
  GraphComps_t comp_n, comp_e;
  GraphComp comp_g, current;

  graph::StatePtr_t n;
  graph::EdgePtr_t e;

  ULong len_edges = 0;
  ULong len_nodes = 0;
  try {
    graph::GraphPtr_t g = graph();

    // Set the graph values
    graph_out = new GraphComp();
    graph_out->name = g->name().c_str();
    graph_out->id = (Long)g->id();

    for (std::size_t i = 0; i < g->nbComponents(); ++i) {
      if (i == g->id()) continue;
      graph::GraphComponentPtr_t gcomponent = g->get(i).lock();
      if (!gcomponent) continue;
      current.name = gcomponent->name().c_str();
      current.id = (Long)gcomponent->id();
      n = HPP_DYNAMIC_PTR_CAST(graph::State, gcomponent);
      e = HPP_DYNAMIC_PTR_CAST(graph::Edge, gcomponent);
      if (n) {
        comp_n.length(len_nodes + 1);
        comp_n[len_nodes] = current;
        len_nodes++;
      } else if (e) {
        comp_e.length(len_edges + 1);
        graph::WaypointEdgePtr_t we =
            HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge, e);
        if (we) {
          current.waypoints.length((ULong)we->nbWaypoints());
          for (std::size_t i = 0; i < we->nbWaypoints(); ++i)
            current.waypoints[(ULong)i] = (ID)we->waypoint(i)->stateTo()->id();
        } else {
          current.waypoints.length(0);
        }
        current.start = (Long)e->stateFrom()->id();
        current.end = (Long)e->stateTo()->id();
        comp_e[len_edges] = current;
        len_edges++;
      }
    }
    elmts = new GraphElements;
    elmts->nodes = comp_n;
    elmts->edges = comp_e;
  } catch (std::out_of_range& e) {
    throw Error(e.what());
  }
}

void Graph::getEdgeStat(ID edgeId, Names_t_out reasons, intSeq_out freqs) {
  try {
    graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId, true);
    core::PathPlannerPtr_t p = problemSolver()->pathPlanner();
    if (!p) throw Error("There is no planner");
    ManipulationPlannerPtr_t mp = HPP_DYNAMIC_PTR_CAST(ManipulationPlanner, p);
    if (!mp) throw Error("The planner must be a ManipulationPlanner");

    StringList_t errors = ManipulationPlanner::errorList();
    ManipulationPlanner::ErrorFreqs_t fes = mp->getEdgeStat(edge);

    Names_t* r_ptr = toNames_t(errors.begin(), errors.end());
    intSeq* f_ptr = toIntSeq(fes.begin(), fes.end());

    reasons = r_ptr;
    freqs = f_ptr;
  } catch (std::out_of_range& e) {
    throw Error(e.what());
  }
}

Long Graph::getFrequencyOfNodeInRoadmap(ID nodeId,
                                        intSeq_out freqPerConnectedComponent) {
  try {
    graph::StatePtr_t state = getComp<graph::State>(nodeId, true);
    // Long nb = graph_->nodeHistogram()->freq(graph::NodeBin(node));
    std::size_t nb = 0;
    const core::ConnectedComponents_t& ccs =
        problemSolver()->roadmap()->connectedComponents();
    core::ConnectedComponents_t::const_iterator _cc;
    std::vector<std::size_t> freqs;
    for (_cc = ccs.begin(); _cc != ccs.end(); ++_cc) {
      manipulation::ConnectedComponentPtr_t cc =
          HPP_DYNAMIC_PTR_CAST(manipulation::ConnectedComponent, *_cc);
      if (!cc) throw Error("Connected component is not of the right type.");
      freqs.push_back(cc->getRoadmapNodes(state).size());
      nb += freqs.back();
    }
    freqPerConnectedComponent = toIntSeq(freqs.begin(), freqs.end());
    return (Long)nb;
  } catch (std::out_of_range& e) {
    throw Error(e.what());
  }
}

bool Graph::getConfigProjectorStats(ID elmt, ConfigProjStat_out config,
                                    ConfigProjStat_out path) {
  try {
    graph::StatePtr_t state = getComp<graph::State>(elmt, false);
    graph::EdgePtr_t edge = getComp<graph::Edge>(elmt, false);
    if (state) {
      ConfigProjectorPtr_t proj =
          graph()->configConstraint(state)->configProjector();
      if (proj) {
        config.success = (Long)proj->statistics().nbSuccess();
        config.error = (Long)proj->statistics().nbFailure();
        config.nbObs = (Long)proj->statistics().numberOfObservations();
      }
      path.success = 0;
      path.error = 0;
      path.nbObs = 0;
      return true;
    } else if (edge) {
      ConfigProjectorPtr_t proj =
          graph()->targetConstraint(edge)->configProjector();
      if (proj) {
        config.success = (Long)proj->statistics().nbSuccess();
        config.error = (Long)proj->statistics().nbFailure();
        config.nbObs = (Long)proj->statistics().numberOfObservations();
      }
      proj = graph()->pathConstraint(edge)->configProjector();
      if (proj) {
        path.success = (Long)proj->statistics().nbSuccess();
        path.error = (Long)proj->statistics().nbFailure();
        path.nbObs = (Long)proj->statistics().numberOfObservations();
      }
      return true;
    } else {
      throw Error("The ID does not exist.");
    }
    return false;
  } catch (std::out_of_range& e) {
    throw Error(e.what());
  }
}

Long Graph::getWaypoint(const Long edgeId, const Long index,
                        hpp::ID_out nodeId) {
  try {
    graph::WaypointEdgePtr_t edge = getComp<graph::WaypointEdge>(edgeId);

    if (index < 0 || (std::size_t)index > edge->nbWaypoints())
      throw Error("Invalid index");
    graph::EdgePtr_t waypoint = edge->waypoint(index);
    nodeId = (Long)waypoint->stateTo()->id();
    return (Long)waypoint->id();
  } catch (std::out_of_range& e) {
    throw Error(e.what());
  }
}

Long Graph::createLevelSetEdge(const Long nodeFromId, const Long nodeToId,
                               const char* edgeName, const Long w,
                               const ID isInNodeId) {
  try {
    graph::StatePtr_t from = getComp<graph::State>(nodeFromId),
                      to = getComp<graph::State>(nodeToId),
                      isInState = getComp<graph::State>(isInNodeId);

    graph::EdgePtr_t edge =
        from->linkTo(edgeName, to, (size_type)w,
                     (graph::State::EdgeFactory)graph::LevelSetEdge::create);

    edge->state(isInState);

    return (Long)edge->id();
  } catch (std::out_of_range& e) {
    throw Error(e.what());
  }
}

void Graph::addLevelSetFoliation(const Long edgeId, const hpp::Names_t& condNC,
                                 const hpp::Names_t& paramNC) {
  try {
    graph::LevelSetEdgePtr_t edge = getComp<graph::LevelSetEdge>(edgeId);
    for (CORBA::ULong i = 0; i < condNC.length(); ++i) {
      std::string name(condNC[i]);
      edge->insertConditionConstraint(
          problemSolver()->numericalConstraints.get(name)->copy());
    }
    for (CORBA::ULong i = 0; i < paramNC.length(); ++i) {
      std::string name(paramNC[i]);
      edge->insertParamConstraint(
          problemSolver()->numericalConstraints.get(name)->copy());
    }
  } catch (std::exception& err) {
    throw Error(err.what());
  }
}

void Graph::setContainingNode(const ID edgeId, const ID nodeId) {
  graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId);
  graph::StatePtr_t state = getComp<graph::State>(nodeId);
  try {
    edge->state(state);
  } catch (std::exception& err) {
    throw Error(err.what());
  }
}

char* Graph::getContainingNode(const ID edgeId) {
  graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId);
  try {
    std::string name(edge->state()->name());
    char* res = new char[name.size() + 1];
    strcpy(res, name.c_str());
    return res;
  } catch (std::exception& err) {
    throw Error(err.what());
  }
}

void Graph::addNumericalConstraints(const Long graphComponentId,
                                    const hpp::Names_t& constraintNames) {
  graph::GraphComponentPtr_t component =
      getComp<graph::GraphComponent>(graphComponentId, true);

  if (constraintNames.length() > 0) {
    try {
      for (CORBA::ULong i = 0; i < constraintNames.length(); ++i) {
        std::string name(constraintNames[i]);
        if (!problemSolver()->numericalConstraint(name)) {
          std::ostringstream os;
          os << "Numerical constraint \"" << name << "\" does not exist.";
          throw Error(os.str().c_str());
        }
        component->addNumericalConstraint(
            problemSolver()->numericalConstraint(name));
      }
    } catch (std::exception& err) {
      throw Error(err.what());
    }
  }
}

void Graph::getNumericalConstraints(const Long graphComponentId,
                                    hpp::Names_t_out names) {
  graph::GraphComponentPtr_t elmt =
      getComp<graph::GraphComponent>(graphComponentId);
  core::NumericalConstraints_t constraints = elmt->numericalConstraints();
  names = new hpp::Names_t;
  names->length((ULong)constraints.size());
  int i = 0;
  for (core::NumericalConstraints_t::iterator it = constraints.begin();
       it != constraints.end(); ++it) {
    (*names)[i] = (*it)->function().name().c_str();
    i++;
  }
}

void Graph::resetConstraints(const Long graphComponentId) {
  graph::GraphComponentPtr_t component =
      getComp<graph::GraphComponent>(graphComponentId, true);
  component->resetNumericalConstraints();
}

void Graph::addNumericalConstraintsForPath(
    const Long nodeId, const hpp::Names_t& constraintNames) {
  graph::StatePtr_t n = getComp<graph::State>(nodeId);

  if (constraintNames.length() > 0) {
    try {
      for (CORBA::ULong i = 0; i < constraintNames.length(); ++i) {
        std::string name(constraintNames[i]);
        n->addNumericalConstraintForPath(
            problemSolver()->numericalConstraint(name)->copy());
      }
    } catch (std::exception& err) {
      throw Error(err.what());
    }
  }
}

void Graph::removeCollisionPairFromEdge(ID edgeId, const char* joint1,
                                        const char* joint2) {
  graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId);

  try {
    using hpp::core::RelativeMotion;
    RelativeMotion::matrix_type m(edge->relativeMotion());
    DevicePtr_t robot = getRobotOrThrow(problemSolver());
    JointIndex i1 = robot->getJointByName(joint1)->index();
    JointIndex i2 = robot->getJointByName(joint2)->index();
    m(i1, i2) = m(i2, i1) = RelativeMotion::Constrained;
    edge->relativeMotion(m);
  } catch (std::exception& err) {
    throw Error(err.what());
  }
}

void Graph::getNode(const hpp::floatSeq& dofArray, ID_out output) {
  DevicePtr_t robot = getRobotOrThrow(problemSolver());
  try {
    Configuration_t config(floatSeqToConfig(robot, dofArray, true));
    graph::StatePtr_t state = graph()->getState(config);
    output = (Long)state->id();
  } catch (std::exception& e) {
    throw Error(e.what());
  }
}

bool Graph::applyNodeConstraints(hpp::ID id, const hpp::floatSeq& input,
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

bool Graph::applyEdgeLeafConstraints(hpp::ID IDedge, const hpp::floatSeq& qleaf,
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
    ConfigurationPtr_t qRhs = floatSeqToConfigPtr(robot, qleaf, true);
    ConstraintSetPtr_t cs(edge->pathConstraint());
    assert(cs);

    if (cs->configProjector()) {
      cs->configProjector()->rightHandSideFromConfig(*qRhs);
      success = cs->apply(*config);
      residualError = cs->configProjector()->residualError();
    } else {
      residualError = 0;
    }

    output = vectorToFloatSeq(*config);
    return success;
  } catch (const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

bool Graph::generateTargetConfig(hpp::ID IDedge, const hpp::floatSeq& qleaf,
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
    ConfigurationPtr_t qRhs = floatSeqToConfigPtr(robot, qleaf, true);
    value_type dist = 0;
    core::NodePtr_t nNode = problemSolver()->roadmap()->nearestNode(qRhs, dist);
    if (dist < 1e-8)
      success = edge->generateTargetConfig(nNode, *config);
    else
      success = edge->generateTargetConfig(*qRhs, *config);

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

CORBA::Boolean Graph::getConfigErrorForNode(ID nodeId,
                                            const hpp::floatSeq& dofArray,
                                            hpp::floatSeq_out error) {
  graph::StatePtr_t state = getComp<graph::State>(nodeId);
  DevicePtr_t robot = getRobotOrThrow(problemSolver());
  try {
    vector_t err;
    Configuration_t config(floatSeqToConfig(robot, dofArray, true));
    bool res = graph()->getConfigErrorForState(config, state, err);
    error = vectorToFloatSeq(err);
    return res;
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

CORBA::Boolean Graph::getConfigErrorForEdge(ID edgeId,
                                            const hpp::floatSeq& dofArray,
                                            hpp::floatSeq_out error) {
  DevicePtr_t robot = getRobotOrThrow(problemSolver());
  try {
    graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId);
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
    vector_t err;
    Configuration_t config(floatSeqToConfig(robot, dofArray, true));
    bool res = graph()->getConfigErrorForEdge(config, edge, err);
    floatSeq* e = new floatSeq();
    e->length((ULong)err.size());
    for (std::size_t i = 0; i < (std::size_t)err.size(); ++i) {
      (*e)[(ULong)i] = err[i];
    }
    error = e;
    return res;
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

CORBA::Boolean Graph::getConfigErrorForEdgeLeaf(
    ID edgeId, const hpp::floatSeq& leafDofArray, const hpp::floatSeq& dofArray,
    hpp::floatSeq_out error) {
  DevicePtr_t robot = getRobotOrThrow(problemSolver());
  try {
    graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId);
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
    vector_t err;
    Configuration_t leafConfig(floatSeqToConfig(robot, leafDofArray, true));
    Configuration_t config(floatSeqToConfig(robot, dofArray, true));
    bool res =
        graph()->getConfigErrorForEdgeLeaf(leafConfig, config, edge, err);
    floatSeq* e = new floatSeq();
    e->length((ULong)err.size());
    for (std::size_t i = 0; i < (std::size_t)err.size(); ++i) {
      (*e)[(ULong)i] = err[i];
    }
    error = e;
    return res;
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

CORBA::Boolean Graph::getConfigErrorForEdgeTarget(
    ID edgeId, const hpp::floatSeq& leafDofArray, const hpp::floatSeq& dofArray,
    hpp::floatSeq_out error) {
  DevicePtr_t robot = getRobotOrThrow(problemSolver());
  try {
    graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId);
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
    vector_t err;
    Configuration_t leafConfig(floatSeqToConfig(robot, leafDofArray, true));
    Configuration_t config(floatSeqToConfig(robot, dofArray, true));
    bool res =
        graph()->getConfigErrorForEdgeTarget(leafConfig, config, edge, err);
    floatSeq* e = new floatSeq();
    e->length((ULong)err.size());
    for (std::size_t i = 0; i < (std::size_t)err.size(); ++i) {
      (*e)[(ULong)i] = err[i];
    }
    error = e;
    return res;
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Graph::displayNodeConstraints(hpp::ID nodeId,
                                   CORBA::String_out constraints) {
  try {
    graph::StatePtr_t state = getComp<graph::State>(nodeId);
    ConstraintSetPtr_t cs(graph()->configConstraint(state));
    std::ostringstream oss;
    oss << (*cs);
    constraints = oss.str().c_str();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Graph::displayEdgeTargetConstraints(hpp::ID edgeId,
                                         CORBA::String_out constraints) {
  try {
    graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId);
    ConstraintSetPtr_t cs(graph()->targetConstraint(edge));
    std::ostringstream oss;
    oss << (*cs);
    constraints = oss.str().c_str();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Graph::displayEdgeConstraints(hpp::ID edgeId,
                                   CORBA::String_out constraints) {
  try {
    graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId);
    ConstraintSetPtr_t cs(graph()->pathConstraint(edge));
    std::ostringstream oss;
    oss << (*cs);
    constraints = oss.str().c_str();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Graph::getNodesConnectedByEdge(hpp::ID edgeId, CORBA::String_out from,
                                    CORBA::String_out to) {
  try {
    graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId);
    from = edge->stateFrom()->name().c_str();
    to = edge->stateTo()->name().c_str();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Graph::display(const char* filename) {
  try {
    std::cout << *graph();
    std::ofstream dotfile;
    dotfile.open(filename);
    graph()->dotPrint(dotfile);
    dotfile.close();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Graph::getHistogramValue(ID edgeId, hpp::floatSeq_out freq,
                              hpp::floatSeqSeq_out values) {
  graph::LevelSetEdgePtr_t edge = getComp<graph::LevelSetEdge>(edgeId);
  try {
    graph::LeafHistogramPtr_t hist = edge->histogram();
    floatSeq* _freq = new floatSeq();
    floatSeqSeq* _values = new floatSeqSeq();
    _freq->length(hist->numberOfBins());
    _values->length(hist->numberOfBins());
    ULong i = 0;
    for (graph::LeafHistogram::const_iterator it = hist->begin();
         it != hist->end(); ++it) {
      (*_freq)[i] = (CORBA::Double)it->freq();
      floatSeq v;
      const vector_t& offset = it->value();
      v.length((ULong)offset.size());
      for (ULong j = 0; j < offset.size(); ++j) v[j] = (CORBA::Double)offset[j];
      (*_values)[i] = v;
      i++;
    }
    freq = _freq;
    values = _values;
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Graph::setShort(ID edgeId, CORBA::Boolean isShort) {
  graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId);
  try {
    edge->setShort(isShort);
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

bool Graph::isShort(ID edgeId) {
  graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId);
  try {
    return edge->isShort();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

Long Graph::autoBuild(const char* graphName, const Names_t& grippers,
                      const Names_t& objects, const Namess_t& handlesPerObject,
                      const Namess_t& shapesPreObject, const Names_t& envNames,
                      const Rules& rulesList) {
  std::vector<graph::helper::Rule> rules(rulesList.length());

  for (ULong i = 0; i < rulesList.length(); ++i) {
    setRule(rulesList[i], rules[i]);
  }
  try {
    graph::GraphPtr_t g = graph::helper::graphBuilder(
        problemSolver(), graphName,
        corbaServer::toStrings<std::vector<std::string> >(grippers),
        toObjectVector(objects, handlesPerObject, shapesPreObject),
        corbaServer::toStrings<std::vector<std::string> >(envNames), rules);

    return (Long)g->id();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Graph::setWeight(ID edgeId, const Long weight) {
  graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId);
  try {
    edge->stateFrom()->updateWeight(edge, weight);
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

Long Graph::getWeight(ID edgeId) {
  graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId);
  try {
    return (Long)edge->stateFrom()->getWeight(edge);
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

char* Graph::getName(ID elmtId) {
  try {
    return corbaServer::c_str(graph()->get(elmtId).lock()->name());
  } catch (std::exception& e) {
    throw Error(e.what());
  }
}

void Graph::initialize() {
  try {
    problemSolver()->initConstraintGraph();
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Graph::getRelativeMotionMatrix(ID edgeId, intSeqSeq_out matrix) {
  try {
    graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId, true);
    matrix = matrixToIntSeqSeq(edge->relativeMotion().cast<CORBA::Long>());
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Graph::getSecurityMarginMatrixForEdge(ID edgeId, floatSeqSeq_out matrix) {
  try {
    graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId, true);
    matrix = corbaServer::matrixToFloatSeqSeq(edge->securityMargins());
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

void Graph::setSecurityMarginForEdge(ID edgeId, const char* joint1,
                                     const char* joint2, double margin) {
  try {
    graph::EdgePtr_t edge = getComp<graph::Edge>(edgeId, true);
    DevicePtr_t robot(edge->parentGraph()->robot());
    JointPtr_t j1(robot->getJointByName(joint1));
    JointPtr_t j2(robot->getJointByName(joint2));
    size_type i1 = 0;
    if (j1) {
      i1 = j1->index();
    }
    size_type i2 = 0;
    if (j2) {
      i2 = j2->index();
    }
    edge->securityMarginForPair(i1, i2, margin);
  } catch (const std::exception& exc) {
    throw Error(exc.what());
  }
}

}  // namespace impl
}  // namespace manipulation
}  // namespace hpp
