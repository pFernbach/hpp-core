//
// Copyright (c) 2015 CNRS
// Authors: Mylene Campana
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#include <boost/tuple/tuple.hpp>
#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/model/device.hh>
#include <hpp/core/parabola/parabola-planner.hh>
#include <hpp/core/node.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/path.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <boost/tuple/tuple.hpp>
#include <hpp/core/contact-configuration-shooter.hh>
#include <hpp/core/configuration-projection-shooter.hh>

namespace hpp {
  namespace core {
    using model::displayConfig;

    ParabolaPlannerPtr_t ParabolaPlanner::createWithRoadmap
    (const Problem& problem, const RoadmapPtr_t& roadmap)
    {
      ParabolaPlanner* ptr = new ParabolaPlanner (problem, roadmap);
      return ParabolaPlannerPtr_t (ptr);
    }

    ParabolaPlannerPtr_t ParabolaPlanner::create (const Problem& problem)
    {
      ParabolaPlanner* ptr = new ParabolaPlanner (problem);
      return ParabolaPlannerPtr_t (ptr);
    }

    ParabolaPlanner::ParabolaPlanner (const Problem& problem):
      PathPlanner (problem)
    {
      configurationShooter (problem.configurationShooter ());
    }

    ParabolaPlanner::ParabolaPlanner (const Problem& problem,
				      const RoadmapPtr_t& roadmap) :
      PathPlanner (problem, roadmap)
    {
      configurationShooter (problem.configurationShooter ());
    }

    void ParabolaPlanner::init (const ParabolaPlannerWkPtr_t& weak)
    {
      PathPlanner::init (weak);
      weakPtr_ = weak;
    }

    void ParabolaPlanner::oneStep ()
    {
      typedef boost::tuple <NodePtr_t, NodePtr_t, PathPtr_t>
	DelayedEdge_t;
      typedef std::vector <DelayedEdge_t> DelayedEdges_t;
      DevicePtr_t robot (problem ().robot ());
      PathValidationPtr_t pathValidation (problem ().pathValidation ());
      ConfigValidationsPtr_t configValidations (problem ().configValidations());
      SteeringMethodPtr_t sm (problem ().steeringMethod ());
      PathPtr_t validPath, validPart, localPath;
      ValidationReportPtr_t validationReport;
      bool validConfig;
      DelayedEdge_t fwdDelayedEdge, bwdDelayedEdge;
      DelayedEdges_t fwdDelayedEdges;
      
      // shoot a valid random configuration at the contact of an obstacle
      ConfigurationPtr_t q_rand;
      do {
	validConfig = false;
	q_rand = configurationShooter_->shoot ();
	if (configValidations->validate (*q_rand, validationReport)) {
	  validConfig = true;
	  hppDout (info, "q_rand: " << displayConfig (*q_rand));
	}
      } while (!validConfig);

      // Add q_rand as a new node: here for the parabola, as the impact node
      core::NodePtr_t impactNode = roadmap ()->addNode (q_rand);

      // try to connect the random configuration to each connected component
      // of the roadmap.
      for (ConnectedComponents_t::const_iterator itcc =
	     roadmap ()->connectedComponents ().begin ();
	   itcc != roadmap ()->connectedComponents ().end (); ++itcc) {
	ConnectedComponentPtr_t cc = *itcc;
	// except its own connected component of course
	if (cc != impactNode->connectedComponent ()) {

	  // iteration on each node of the current connected-component
	  for (Nodes_t::const_iterator n_it = cc->nodes ().begin (); 
	       n_it != cc->nodes ().end (); ++n_it){
	    ConfigurationPtr_t qCC = (*n_it)->configuration ();

	    // Create forward local path from qCC to q_rand
	    localPath = (*sm) (*qCC, *q_rand);

	    // if a forward path is returned, it is valid
	    if (localPath) {
	      // Save forward & backward delayed edges
	      fwdDelayedEdge = DelayedEdge_t (*n_it, impactNode, localPath);
	      fwdDelayedEdges.push_back (fwdDelayedEdge);
		
	      // Assuming that SM is symmetric (V0max = Vfmax)
	      // WARN: I had to reverse *n_it, q_rand HERE
	      // To add edges consecutively to same vector fwdDelayedEdges
	      bwdDelayedEdge = DelayedEdge_t (impactNode, *n_it,
					      localPath->reverse ());
	      fwdDelayedEdges.push_back (bwdDelayedEdge);
	    } //if SM has returned a non-empty path
	  }//for nodes in cc
	}//avoid impactNode cc
      }//for cc in roadmap

      // Insert in roadmap all forward delayed edges (DE)
      for (DelayedEdges_t::const_iterator itEdge = fwdDelayedEdges.begin ();
	   itEdge != fwdDelayedEdges.end (); ++itEdge) {
	const NodePtr_t& nodeDE = itEdge-> get <0> ();
	const NodePtr_t& node2DE = itEdge-> get <1> ();
	const PathPtr_t& pathDE = itEdge-> get <2> ();
	roadmap ()->addEdge (nodeDE, node2DE, pathDE);
	hppDout(info, "connection between q1: " 
		<< displayConfig (*(nodeDE->configuration ()))
		<< "and q2: "
		<< displayConfig (*(impactNode->configuration ())));
      }
    }

    void ParabolaPlanner::configurationShooter
    (const ConfigurationShooterPtr_t& shooter)
    {
      configurationShooter_ = shooter;
    }


  } // namespace core
} // namespace hpp
