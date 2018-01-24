// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see <http://www.gnu.org/licenses/>.

#include "graph.impl.hh"

#include <fstream>
#include <sstream>

#include <boost/foreach.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>
#include <hpp/util/pointer.hh>

#include <hpp/manipulation/problem.hh>
#include <hpp/manipulation/roadmap.hh>
#include <hpp/manipulation/connected-component.hh>
#include <hpp/manipulation/manipulation-planner.hh>
#include <hpp/manipulation/graph/state-selector.hh>
#include <hpp/manipulation/graph/guided-state-selector.hh>
#include <hpp/manipulation/graph/state.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/helper.hh>
#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/graph-steering-method.hh>

#include <hpp/constraints/differentiable-function.hh>

#include <hpp/corbaserver/manipulation/server.hh>

#include "tools.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      using CORBA::ULong;
      using graph::Edge;
      using graph::LevelSetEdge;
      using graph::WaypointEdge;
      using graph::EdgePtr_t;
      using graph::LevelSetEdgePtr_t;
      using graph::WaypointEdgePtr_t;

      using corbaServer::floatSeqToConfig;

      namespace {
        typedef core::ProblemSolver CPs_t;

        template <typename T> std::string toStr () { return typeid(T).name(); }
        template <> std::string toStr <graph::State> () { return "Node"; }
        template <> std::string toStr <graph::Edge> () { return "Edge"; }
        template <> std::string toStr <graph::Graph> () { return "Graph"; }
        template <> std::string toStr <graph::StateSelector> () { return "SubGraph"; }
        template <> std::string toStr <graph::GuidedStateSelector> () { return "Guided Node Selector"; }
        template <> std::string toStr <graph::LevelSetEdge> () { return "LevelSetEdge"; }
        template <> std::string toStr <graph::WaypointEdge> () { return "WaypointEdge"; }

        std::list<graph::helper::ObjectDef_t> toObjectList (
            const Names_t& names, const Namess_t& hsPO, const Namess_t& shPO) {
          using graph::helper::ObjectDef_t;
          std::list<graph::helper::ObjectDef_t> ret;
	  // Check size of lists
	  if (hsPO.length () != names.length ()) {
            HPP_THROW(Error, "Number of handle lists (" << hsPO.length ()
		<< ") does not match number of objects (" << names.length ()
		<< ").");
	  }
	  if (shPO.length () != names.length ()) {
            HPP_THROW(Error, "Number of contact lists (" << shPO.length ()
		<< ") does not match number of objects (" << names.length ()
		<< ").");
	  }
          for (ULong i = 0; i < names.length(); ++i) {
            ret.push_back (ObjectDef_t());
            ObjectDef_t& od = ret.back ();
            od.name = names[i];
            od.handles = toStringList (hsPO[i]);
            od.shapes = toStringList (shPO[i]);
          }
          return ret;
        }

        void setRule (const hpp::corbaserver::manipulation::Rule& in, graph::helper::Rule& out)
        {
          out.grippers_ = toStringVector (in.grippers);
          out.handles_  = toStringVector (in.handles );
          out.link_ = in.link;
        }
      }

      std::vector <std::string>
      convertPassiveDofNameVector (const hpp::Names_t& names, const size_t& s)
      {
	if (names.length () != s) {
	  std::ostringstream oss;
	  oss << "Number of constraints (" << s
	      << ") and number of lists of passive joints (" <<
	    names.length () << ") should be the same."
	      << std::endl;
	  oss << "Use ProblemSolver.addPassiveDofs to create lists of passive joints.";
	    throw std::runtime_error (oss.str ().c_str ());
	}
        return toStringVector (names);
      }

      Graph::Graph () :
        server_ (0x0)
      {}

      ProblemSolverPtr_t Graph::problemSolver ()
      {
        return server_->problemSolver();
      }

      graph::GraphPtr_t Graph::graph (bool throwIfNull)
      {
        graph::GraphPtr_t g = problemSolver()->constraintGraph();
        if (throwIfNull && !g)
          throw Error ("You should create the graph");
        return g;
      }

      template <typename T> boost::shared_ptr<T> Graph::getComp (ID id, bool throwIfWrongType)
      { 
        boost::shared_ptr <T> comp;
        try {
          comp = HPP_DYNAMIC_PTR_CAST(T, graph()->get(id).lock());
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
        if (throwIfWrongType && !comp) {
          std::stringstream ss;
          ss << "ID " << id << " is not a " << toStr <T>();
          throw Error (ss.str().c_str());
        }
        return comp;
      }

      Long Graph::createGraph(const char* graphName)
        throw (hpp::Error)
      {
        DevicePtr_t robot = problemSolver()->robot ();
        if (!robot) throw Error ("Build the robot first.");
	// Create default steering method to store in edges, until we define a
	// factory for steering methods.
        graph::GraphPtr_t g = graph::Graph::create(graphName, robot,
            problemSolver()->problem());
        g->maxIterations (problemSolver()->maxIterProjection ());
        g->errorThreshold (problemSolver()->errorThreshold ());
        problemSolver()->constraintGraph (g);
        problemSolver()->problem()->constraintGraph (g);
        return (Long) g->id ();
      }

      Long Graph::createSubGraph(const char* subgraphName)
        throw (hpp::Error)
      {
        graph::GuidedStateSelectorPtr_t ns = graph::GuidedStateSelector::create
          (subgraphName, problemSolver()->roadmap ());
        graph()->stateSelector(ns);
        return (Long) ns->id ();
      }

      void Graph::setTargetNodeList(const ID subgraph, const hpp::IDseq& nodes)
        throw (hpp::Error)
      {
        graph::GuidedStateSelectorPtr_t ns = getComp <graph::GuidedStateSelector> (subgraph);
        try {
          graph::States_t nl;
          for (unsigned int i = 0; i < nodes.length(); ++i)
            nl.push_back (getComp <graph::State> (nodes[i]));
          ns->setStateList (nl);
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
      }

      Long Graph::createNode(const Long subgraphId, const char* nodeName,
          const bool waypoint, const Long priority)
        throw (hpp::Error)
      {
        graph::StateSelectorPtr_t ns = getComp <graph::StateSelector> (subgraphId);

        graph::StatePtr_t state = ns->createState (nodeName, waypoint, priority);
        return (Long) state->id ();
      }

      Long Graph::createEdge(const Long nodeFromId, const Long nodeToId, const char* edgeName, const Long w, const Long isInNodeId)
        throw (hpp::Error)
      {
        graph::StatePtr_t from = getComp <graph::State> (nodeFromId),
	  to = getComp <graph::State> (nodeToId),
	  isInState = getComp <graph::State> (isInNodeId);

        EdgePtr_t edge = from->linkTo
          (edgeName, to, (size_type)w, (graph::State::EdgeFactory)Edge::create);
	edge->state (isInState);

        return (Long) edge->id ();
      }

      Long Graph::createWaypointEdge(const Long nodeFromId, const Long nodeToId,
          const char* edgeName, const Long nb, const Long w,
          const Long isInNodeId)
        throw (hpp::Error)
      {
        graph::StatePtr_t from = getComp <graph::State> (nodeFromId),
	  to = getComp <graph::State> (nodeToId),
	  isInNode = getComp <graph::State> (isInNodeId);

        EdgePtr_t edge_pc = from->linkTo
          (edgeName, to, (size_type)w,
           (graph::State::EdgeFactory)WaypointEdge::create);

        edge_pc->state (isInNode);
        WaypointEdgePtr_t edge = HPP_DYNAMIC_PTR_CAST (WaypointEdge, edge_pc);

        edge->nbWaypoints (nb);
        return (Long) edge->id ();
      }

      void Graph::setWaypoint (const ID waypointEdgeId, const Long index,
          const ID edgeId, const ID nodeId)
        throw (hpp::Error)
      {
        WaypointEdgePtr_t we = getComp <graph::WaypointEdge> (waypointEdgeId);
        EdgePtr_t edge = getComp <Edge> (edgeId);
        graph::StatePtr_t state = getComp <graph::State> (nodeId);

        if (index < 0 || (std::size_t)index > we->nbWaypoints ())
          throw Error ("Invalid index");
        we->setWaypoint (index, edge, state);
      }

      void Graph::getGraph (GraphComp_out graph_out, GraphElements_out elmts)
        throw (hpp::Error)
      {
        graph::GraphPtr_t g = graph();
        GraphComps_t comp_n, comp_e;
        GraphComp comp_g, current;

        graph::StatePtr_t n;
        graph::EdgePtr_t e;

        ULong len_edges = 0;
        ULong len_nodes = 0;
        try {
          // Set the graph values
          graph_out = new GraphComp ();
          graph_out->name = g->name ().c_str();
          graph_out->id = (Long) g->id ();

          for (std::size_t i = 0; i < g->nbComponents(); ++i) {
            if (i == g->id ()) continue;
            graph::GraphComponentPtr_t gcomponent = g->get(i).lock();
            if (!gcomponent) continue;
            current.name = gcomponent->name ().c_str ();
            current.id   = (Long) gcomponent->id ();
            n = HPP_DYNAMIC_PTR_CAST(graph::State, gcomponent);
            e = HPP_DYNAMIC_PTR_CAST(graph::Edge, gcomponent);
            if (n) {
              comp_n.length (len_nodes + 1);
              comp_n[len_nodes] = current;
              len_nodes++;
            } else if (e) {
              comp_e.length (len_edges + 1);
              graph::WaypointEdgePtr_t we = HPP_DYNAMIC_PTR_CAST (
                  graph::WaypointEdge, e);
              if (we) {
                current.waypoints.length((ULong)we->nbWaypoints());
                for (std::size_t i = 0; i < we->nbWaypoints(); ++i)
                  current.waypoints[(ULong)i] = (ID)we->waypoint(i)->to()->id();
              } else {
                current.waypoints.length(0);
              }
              current.start = (Long) e->from ()->id ();
              current.end = (Long) e->to ()->id ();
              comp_e[len_edges] = current;
              len_edges++;
            }
          }
          elmts = new GraphElements;
          elmts->nodes = comp_n;
          elmts->edges = comp_e;
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
      }

      void Graph::getEdgeStat (ID edgeId, Names_t_out reasons, intSeq_out freqs)
        throw (hpp::Error)
      {
        graph::EdgePtr_t edge = getComp <graph::Edge> (edgeId, true);
        core::PathPlannerPtr_t p = problemSolver()->pathPlanner ();
        if (!p) throw Error ("There is no planner");
        ManipulationPlannerPtr_t mp =
          HPP_DYNAMIC_PTR_CAST (ManipulationPlanner, p);
        if (!mp) throw Error ("The planner must be a ManipulationPlanner");

        StringList_t errors = ManipulationPlanner::errorList ();
        ManipulationPlanner::ErrorFreqs_t fes = mp->getEdgeStat (edge);

        Names_t *r_ptr = toNames_t (errors.begin (), errors.end());
        intSeq *f_ptr = toIntSeq (fes.begin (), fes.end());

        reasons = r_ptr;
        freqs = f_ptr;
      }

      Long Graph::getFrequencyOfNodeInRoadmap (ID nodeId, intSeq_out freqPerConnectedComponent)
        throw (hpp::Error)
      {
        graph::StatePtr_t state = getComp <graph::State> (nodeId, true);
        // Long nb = graph_->nodeHistogram()->freq(graph::NodeBin(node));
        std::size_t nb = 0;
        const core::ConnectedComponents_t& ccs = problemSolver()->roadmap ()->connectedComponents();
        core::ConnectedComponents_t::const_iterator _cc;
        std::vector<std::size_t> freqs;
        for (_cc = ccs.begin(); _cc != ccs.end(); ++_cc) {
          manipulation::ConnectedComponentPtr_t cc =
            HPP_DYNAMIC_PTR_CAST(manipulation::ConnectedComponent, *_cc);
          if (!cc)
            throw Error ("Connected component is not of the right type.");
          freqs.push_back(cc->getRoadmapNodes(state).size());
          nb += freqs.back();
        }
        freqPerConnectedComponent = toIntSeq(freqs.begin(), freqs.end());
        return (Long) nb;
      }

      bool Graph::getConfigProjectorStats (ID elmt, ConfigProjStat_out config,
          ConfigProjStat_out path)
        throw (hpp::Error)
      {
        graph::StatePtr_t state = getComp <graph::State> (elmt, false);
        graph::EdgePtr_t edge = getComp <graph::Edge> (elmt, false);
        if (state) {
          ConfigProjectorPtr_t proj =
            graph()->configConstraint (state)->configProjector ();
          if (proj) {
            config.success = (Long) proj->statistics().nbSuccess();
            config.error = (Long) proj->statistics().nbFailure();
            config.nbObs = (Long) proj->statistics().numberOfObservations();
          }
          path.success = 0;
          path.error = 0;
          path.nbObs = 0;
          return true;
        } else if (edge) {
          ConfigProjectorPtr_t proj =
            graph()->configConstraint (edge)->configProjector ();
          if (proj) {
            config.success = (Long) proj->statistics().nbSuccess();
            config.error = (Long) proj->statistics().nbFailure();
            config.nbObs = (Long) proj->statistics().numberOfObservations();
          }
          proj = graph()->pathConstraint (edge)->configProjector ();
          if (proj) {
            path.success = (Long) proj->statistics().nbSuccess();
            path.error = (Long) proj->statistics().nbFailure();
            path.nbObs = (Long) proj->statistics().numberOfObservations();
          }
          return true;
        } else {
          throw Error ("The ID does not exist.");
        }
        return false;
      }

      Long Graph::getWaypoint (const Long edgeId, const Long index,
          hpp::ID_out nodeId)
        throw (hpp::Error)
      {
        graph::WaypointEdgePtr_t edge = getComp <graph::WaypointEdge> (edgeId);

        if (index < 0 || (std::size_t)index > edge->nbWaypoints ())
          throw Error ("Invalid index");
        graph::EdgePtr_t waypoint = edge->waypoint (index);
        nodeId = (Long) waypoint->to ()->id ();
        return (Long) waypoint->id ();
      }

      Long Graph::createLevelSetEdge(const Long nodeFromId, const Long nodeToId, const char* edgeName, const Long w, const ID isInNodeId)
        throw (hpp::Error)
      {
        graph::StatePtr_t from      = getComp <graph::State> (nodeFromId),
                          to        = getComp <graph::State> (nodeToId  ),
	                  isInState = getComp <graph::State> (isInNodeId);

        graph::EdgePtr_t edge = from->linkTo
          (edgeName, to, (size_type)w,
           (graph::State::EdgeFactory)graph::LevelSetEdge::create);

        edge->state (isInState);

        return (Long) edge->id ();
      }

      void Graph::addLevelSetFoliation (const Long edgeId,
          const hpp::Names_t& condNC, const hpp::Names_t& condLJ,
          const hpp::Names_t& paramNC, const hpp::Names_t& paramPDOF,
          const hpp::Names_t& paramLJ)
        throw (hpp::Error)
      {
        graph::LevelSetEdgePtr_t edge = getComp <graph::LevelSetEdge> (edgeId);
        try {
          for (CORBA::ULong i=0; i<condNC.length (); ++i) {
            std::string name (condNC [i]);
            edge->insertConditionConstraint
              (HPP_STATIC_PTR_CAST(NumericalConstraint,
                                   problemSolver()->get
                                   <NumericalConstraintPtr_t>(name)->copy ()));
          }
          for (CORBA::ULong i=0; i<condLJ.length (); ++i) {
            std::string name (condLJ [i]);
            edge->insertConditionConstraint (problemSolver()->get
                                             <LockedJointPtr_t> (name));
          }

          std::vector <std::string> pdofNames = convertPassiveDofNameVector
            (paramPDOF, paramNC.length ());
          for (CORBA::ULong i=0; i<paramNC.length (); ++i) {
            std::string name (paramNC [i]);
            edge->insertParamConstraint (
                HPP_STATIC_PTR_CAST(NumericalConstraint,
                problemSolver()->get <NumericalConstraintPtr_t>(name)->copy ()),
                problemSolver()->passiveDofs (pdofNames [i]));
          }
          for (CORBA::ULong i=0; i<paramLJ.length (); ++i) {
            std::string name (paramLJ [i]);
            edge->insertParamConstraint
              (problemSolver()->get <LockedJointPtr_t> (name));
          }

          // edge->buildHistogram ();
        } catch (std::exception& err) {
          throw Error (err.what());
        }
      }

      void Graph::setContainingNode (const ID edgeId, const ID nodeId)
        throw (hpp::Error)
      {
        graph::EdgePtr_t edge = getComp <graph::Edge> (edgeId);
        graph::StatePtr_t state = getComp <graph::State> (nodeId);
        try {
          edge->state (state);
        } catch (std::exception& err) {
          throw Error (err.what());
        }
      }

      char* Graph::getContainingNode (const ID edgeId)
            throw (hpp::Error)
      {
        graph::EdgePtr_t edge = getComp <graph::Edge> (edgeId);
        try {
	  std::string name (edge->state ()->name ());
	  char* res = new char [name.size () + 1];
	  strcpy (res, name.c_str ());
	  return res;
        } catch (std::exception& err) {
          throw Error (err.what());
        }
      }

      void Graph::addNumericalConstraints (const Long graphComponentId,
          const hpp::Names_t& constraintNames,
          const hpp::Names_t& passiveDofsNames)
        throw (hpp::Error)
      {
        graph::GraphComponentPtr_t component = getComp<graph::GraphComponent>(graphComponentId, true);

        if (constraintNames.length () > 0) {
          try {
            std::vector <std::string> pdofNames = convertPassiveDofNameVector
              (passiveDofsNames, constraintNames.length ());
            for (CORBA::ULong i=0; i<constraintNames.length (); ++i) {
              std::string name (constraintNames [i]);
              if (!problemSolver()->numericalConstraint (name))
                throw Error ("The numerical function does not exist.");
              component->addNumericalConstraint
		(problemSolver()->numericalConstraint(name),
		 problemSolver()->passiveDofs (pdofNames [i]));
            }
          } catch (std::exception& err) {
            throw Error (err.what());
          }
        }
      }

      void Graph::getNumericalConstraints(const Long graphComponentId, hpp::Names_t_out names)
	throw(hpp::Error)
      {
	graph::GraphComponentPtr_t elmt = getComp<graph::GraphComponent>(graphComponentId);
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

      void Graph::getLockedJoints(const Long graphComponentId, hpp::Names_t_out names)
	throw(hpp::Error)
      {
	graph::GraphComponentPtr_t elmt = getComp<graph::GraphComponent>(graphComponentId, true);
	core::LockedJoints_t lockedJoints = elmt->lockedJoints();
	names = new hpp::Names_t;
	names->length((ULong)lockedJoints.size());
	int i = 0;
	for (core::LockedJoints_t::iterator it = lockedJoints.begin();
	     it != lockedJoints.end(); ++it) {
	  (*names)[i] = (*it)->jointName().c_str();
	  i++;
	}
      }

      void Graph::resetConstraints(const Long graphComponentId) throw (hpp::Error)
      {
        graph::GraphComponentPtr_t component =
          getComp<graph::GraphComponent>(graphComponentId, true);
	component->resetNumericalConstraints();
	component->resetLockedJoints();
      }

      void Graph::addNumericalConstraintsForPath (const Long nodeId,
          const hpp::Names_t& constraintNames,
          const hpp::Names_t& passiveDofsNames)
        throw (hpp::Error)
      {
        graph::StatePtr_t n = getComp <graph::State> (nodeId);

        if (constraintNames.length () > 0) {
          try {
            std::vector <std::string> pdofNames = convertPassiveDofNameVector
              (passiveDofsNames, constraintNames.length ());
            for (CORBA::ULong i=0; i<constraintNames.length (); ++i) {
              std::string name (constraintNames [i]);
              n->addNumericalConstraintForPath
		(HPP_STATIC_PTR_CAST
		 (NumericalConstraint,
		  problemSolver()->numericalConstraint(name)->copy ()),
		 problemSolver()->passiveDofs (pdofNames [i]));
            }
          } catch (std::exception& err) {
            throw Error (err.what());
          }
        }
      }

      void Graph::addLockedDofConstraints (const Long graphComponentId,
          const hpp::Names_t& constraintNames)
        throw (hpp::Error)
      {
        graph::GraphComponentPtr_t component = getComp<graph::GraphComponent>(graphComponentId, true);

        if (constraintNames.length () > 0) {
          try {
            for (CORBA::ULong i=0; i<constraintNames.length (); ++i) {
              std::string name (constraintNames [i]);
              component->addLockedJointConstraint
		(problemSolver()->get <LockedJointPtr_t> (name));
            }
          } catch (std::exception& err) {
            throw Error (err.what());
          }
        }
      }

      void Graph::getNode (const hpp::floatSeq& dofArray, ID_out output)
        throw (hpp::Error)
      {
        DevicePtr_t robot = getRobotOrThrow (problemSolver());
        try {
          Configuration_t config (floatSeqToConfig (robot, dofArray, true));
          graph::StatePtr_t state = graph()->getState (config);
          output = (Long) state->id();
        } catch (std::exception& e) {
          throw Error (e.what());
        }
      }

      CORBA::Boolean Graph::getConfigErrorForNode
      (ID nodeId, const hpp::floatSeq& dofArray, hpp::floatSeq_out error)
	throw (hpp::Error)
      {
	graph::StatePtr_t state = getComp <graph::State> (nodeId);
        DevicePtr_t robot = getRobotOrThrow (problemSolver());
	try {
	  vector_t err;
          Configuration_t config (floatSeqToConfig (robot, dofArray, true));
	  bool res = graph()->getConfigErrorForState (config, state, err);
	  error = vectorToFloatSeq(err);
	  return res;
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      CORBA::Boolean Graph::getConfigErrorForEdge
      (ID edgeId, const hpp::floatSeq& dofArray, hpp::floatSeq_out error)
	throw (hpp::Error)
      {
        DevicePtr_t robot = getRobotOrThrow (problemSolver());
	try {
	  graph::EdgePtr_t edge = getComp <graph::Edge> (edgeId);
	  // If steering method is not completely set in the graph, create
	  // one.
	  if (!edge->parentGraph ()->problem ()->steeringMethod () ||
	      !edge->parentGraph ()->problem ()->steeringMethod ()
	      ->innerSteeringMethod()) {
	    problemSolver ()->initSteeringMethod ();
	  }
	  vector_t err;
          Configuration_t config (floatSeqToConfig (robot, dofArray, true));
	  bool res = graph()->getConfigErrorForEdge (config, edge, err);
	  floatSeq* e = new floatSeq ();
	  e->length ((ULong) err.size ());
	  for (std::size_t i=0; i < (std::size_t) err.size (); ++i) {
	    (*e) [(ULong) i] = err [i];
	  }
	  error = e;
	  return res;
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      CORBA::Boolean Graph::getConfigErrorForEdgeLeaf
      (ID edgeId, const hpp::floatSeq& leafDofArray,
       const hpp::floatSeq& dofArray, hpp::floatSeq_out error)
	throw (hpp::Error)
      {
        DevicePtr_t robot = getRobotOrThrow (problemSolver());
	try {
	  graph::EdgePtr_t edge = getComp <graph::Edge> (edgeId);
	  // If steering method is not completely set in the graph, create
	  // one.
	  if (!edge->parentGraph ()->problem ()->steeringMethod () ||
	      !edge->parentGraph ()->problem ()->steeringMethod ()
	      ->innerSteeringMethod()) {
	    problemSolver ()->initSteeringMethod ();
	  }
	  vector_t err;
          Configuration_t leafConfig (floatSeqToConfig (robot, leafDofArray, true));
          Configuration_t config (floatSeqToConfig (robot, dofArray, true));
	  bool res = graph()->getConfigErrorForEdgeLeaf
	    (leafConfig, config, edge, err);
	  floatSeq* e = new floatSeq ();
	  e->length ((ULong) err.size ());
	  for (std::size_t i=0; i < (std::size_t) err.size (); ++i) {
	    (*e) [(ULong) i] = err [i];
	  }
	  error = e;
	  return res;
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Graph::displayNodeConstraints
      (hpp::ID nodeId, CORBA::String_out constraints) throw (Error)
      {
	graph::StatePtr_t state = getComp <graph::State> (nodeId);
	ConstraintSetPtr_t cs (graph()->configConstraint (state));
	std::ostringstream oss;
	oss << (*cs);
	constraints = oss.str ().c_str ();
      }

      void Graph::displayEdgeTargetConstraints
      (hpp::ID edgeId, CORBA::String_out constraints) throw (Error)
      {
	graph::EdgePtr_t edge = getComp <graph::Edge> (edgeId);
	ConstraintSetPtr_t cs (graph()->configConstraint (edge));
	std::ostringstream oss;
	oss << (*cs);
	constraints = oss.str ().c_str ();
      }

      void Graph::displayEdgeConstraints
      (hpp::ID edgeId, CORBA::String_out constraints) throw (Error)
      {
	graph::EdgePtr_t edge = getComp <graph::Edge> (edgeId);
	ConstraintSetPtr_t cs (graph()->pathConstraint (edge));
	std::ostringstream oss;
	oss << (*cs);
	constraints = oss.str ().c_str ();
      }

       void Graph::getNodesConnectedByEdge
       (hpp::ID edgeId, CORBA::String_out from, CORBA::String_out to)
	 throw (Error)
       {
	 graph::EdgePtr_t edge = getComp <graph::Edge> (edgeId);
	 from = edge->from ()->name ().c_str ();
	 to = edge->to ()->name ().c_str ();
       }

      void Graph::display (const char* filename)
        throw (hpp::Error)
      {
        std::cout << *graph();
        std::ofstream dotfile;
        dotfile.open (filename);
        graph()->dotPrint (dotfile);
        dotfile.close();
      }

      void Graph::getHistogramValue (ID edgeId, hpp::floatSeq_out freq,
          hpp::floatSeqSeq_out values)
        throw (hpp::Error)
      {
        graph::LevelSetEdgePtr_t edge = getComp <graph::LevelSetEdge> (edgeId);
        try {
          graph::LeafHistogramPtr_t hist = edge->histogram ();
	  floatSeq* _freq = new floatSeq ();
          floatSeqSeq *_values = new floatSeqSeq ();
          _freq->length (hist->numberOfBins ());
          _values->length (hist->numberOfBins ());
          ULong i = 0;
          for (graph::LeafHistogram::const_iterator it = hist->begin ();
              it != hist->end (); ++it) {
            (*_freq)[i] = (CORBA::Double) it->freq ();
            floatSeq v;
            const vector_t& offset = it->value();
            v.length ((ULong)offset.size());
            for (ULong j = 0; j < offset.size(); ++j)
              v[j] = (CORBA::Double) offset [j];
            (*_values)[i] = v;
            i++;
          }
          freq = _freq;
          values = _values;
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Graph::setShort (ID edgeId, CORBA::Boolean isShort)
        throw (hpp::Error)
      {
        graph::EdgePtr_t edge = getComp <graph::Edge> (edgeId);
        try {
          edge->setShort (isShort);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      bool Graph::isShort (ID edgeId)
        throw (hpp::Error)
      {
        graph::EdgePtr_t edge = getComp <graph::Edge> (edgeId);
        try {
          return edge->isShort ();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      intSeq* Graph::autoBuild (
          const char* graphName,
          const Names_t& grippers,
          const Names_t& objects,
          const Namess_t& handlesPerObject,
          const Namess_t& shapesPreObject,
          const Names_t& envNames,
	  const Rules& rulesList)
        throw (hpp::Error)
      {
	std::vector<graph::helper::Rule> rules(rulesList.length());

	for (ULong i = 0; i < rulesList.length(); ++i) {
          setRule (rulesList[i], rules[i]);
	}
        try {
          graph::GraphPtr_t g = graph::helper::graphBuilder (
              problemSolver(),
              graphName,
              toStringList (grippers),
              toObjectList (objects, handlesPerObject, shapesPreObject),
              toStringList (envNames),
              rules
              );
          problemSolver()->constraintGraph (g);
          problemSolver()->problem()->constraintGraph (g);

          std::vector<std::size_t> ids (2);
          ids[0] = g->id();
          ids[1] = g->stateSelector()->id();
          return toIntSeq (ids.begin(), ids.end());
        } catch (const std::exception& exc) {
          throw Error (exc.what ());
        }
      }

      void Graph::setWeight (ID edgeId, const Long weight)
        throw (hpp::Error)
      {
        graph::EdgePtr_t edge = getComp <graph::Edge> (edgeId);
        try {
          edge->from()->updateWeight (edge, weight);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      Long Graph::getWeight (ID edgeId)
        throw (hpp::Error)
      {
        graph::EdgePtr_t edge = getComp <graph::Edge> (edgeId);
        try {
          return (Long) edge->from ()->getWeight (edge);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Graph::initialize ()
        throw (hpp::Error)
      {
        try {
          problemSolver ()->initConstraintGraph ();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Graph::getRelativeMotionMatrix (ID edgeId, intSeqSeq_out matrix)
        throw (hpp::Error)
      {
        graph::EdgePtr_t edge = getComp <graph::Edge> (edgeId, true);
        matrix = matrixToIntSeqSeq(edge->relativeMotion().cast<CORBA::Long>());
      }
    } // namespace impl
  } // namespace manipulation
} // namespace hpp
