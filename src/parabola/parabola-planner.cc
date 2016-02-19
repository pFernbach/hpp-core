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
#include <hpp/core/config-projector.hh>
#include <hpp/model/device.hh>
#include <hpp/core/parabola/parabola-planner.hh>
#include <hpp/core/node.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-validation.hh>
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
      typedef boost::tuple <NodePtr_t, ConfigurationPtr_t, PathPtr_t>
	DelayedEdge_t;
      typedef std::vector <DelayedEdge_t> DelayedEdges_t;
      DevicePtr_t robot (problem ().robot ());
      PathValidationPtr_t pathValidation (problem ().pathValidation ());
      ConfigValidationsPtr_t configValidations (problem ().configValidations());
      SteeringMethodPtr_t sm (problem ().steeringMethod ());
      PathPtr_t validPath, validPart, localPath;
      ValidationReportPtr_t validationReport;
      PathValidationReportPtr_t report;
      bool validConfig;
      DelayedEdge_t fwdDelayedEdge, bwdDelayedEdge;
      DelayedEdges_t fwdDelayedEdges, bwdDelayedEdges;
      /*const size_type index = robot->configSize()
	- robot->extraConfigSpace ().dimension ();*/
      
      // shoot a valid random configuration
      // and try to project this configuration at the contact of an obstacle
      ConfigurationPtr_t qrand;
      do {
	validConfig = false;
	qrand = configurationShooter_->shoot ();
	if (configValidations->validate (*qrand, validationReport)) {
	  validConfig = true;
	  hppDout (info, "qrand: " << displayConfig (*qrand));
	}
      } while (!validConfig);
      // actually, q_rand is already at the contact so no projection needed
      ConfigurationPtr_t q_proj ( new Configuration_t (*qrand));
      hppDout (info, "q_proj: " << displayConfig (*q_proj));

      // Add q_proj as a new node: here for the parabola, as the impact node
      core::NodePtr_t impactNode = roadmap ()->addNode (q_proj);

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

	    // Create forward local path from qCC to q_proj
	    localPath = (*sm) (*qCC, *q_proj);

	    // validate forward local path
	    if (localPath) {
	      if (pathValidation->validate (localPath, false,
					    validPart, report)) {
		hppDout (info, "forward path from SM is valid");
		// Save node from current cc that leads to shortest path
		//if (localPath->length () < lengthFwd) {
		//fwdEdgeExists = true;
		//lengthFwd = localPath->length ();
		// Save forward delayed edge from cc with shorter path
		fwdDelayedEdge = DelayedEdge_t (*n_it, q_proj, localPath);
		hppDout (info, "forward delayed edge is saved and pushed");
		fwdDelayedEdges.push_back (fwdDelayedEdge);

		// Assuming that SM is symmetric (V0max = Vfmax)
		bwdDelayedEdge = DelayedEdge_t (*n_it, q_proj,
						localPath->reverse ());
		hppDout (info, "backward delayed edge is saved and pushed");
		bwdDelayedEdges.push_back (bwdDelayedEdge);

		//}
	      } else {
		problem ().parabolaResults_ [0] ++;
		hppDout (info, "parabola has collisions");
	      }
	      /* Useless bwd part since SM is symmetric
	      // Create backward local path from q_proj to qCC
	      // IDEA: could be avoided when we already know that planeTheta
	      // is not intersecting both cones
	      localPath = (*sm) (*q_proj, *qCC);

	      // validate backward local path
	      if (localPath && pathValidation->validate (localPath, false,
	      validPart, report)) {
	      hppDout (info, "backward path from SM is valid");
	      // Save node from current cc that leads to shortest path
	      //if (localPath->length () < lengthBwd) {
	      //bwdEdgeExists = true;
	      //lengthBwd = localPath->length ();
	      // Save backward delayed edge from cc with shorter path
	      // keeping n_it* information for later edge adding...
	      bwdDelayedEdge = DelayedEdge_t (*n_it, q_proj, localPath);
	      hppDout (info, "backward delayed edge is saved and pushed");
	      bwdDelayedEdges.push_back (bwdDelayedEdge);
	      //}
	      }
	      */
	    } //if SM has returned a non-empty path
	  }//for nodes in cc
	  //if (fwdEdgeExists) // avoid adding a null delayed-edge ...
	  //fwdDelayedEdges.push_back (fwdDelayedEdge); // shortest path from cc
	  //if (bwdEdgeExists) // avoid adding a null delayed-edge ...
	  //bwdDelayedEdges.push_back (bwdDelayedEdge); // shortest path from cc
	  
	}//avoid impactNode cc
      }//for cc in roadmap

      // Insert in roadmap all forward delayed edges (DE)
      for (DelayedEdges_t::const_iterator itEdge = fwdDelayedEdges.begin ();
	   itEdge != fwdDelayedEdges.end (); ++itEdge) {
	const NodePtr_t& nodeDE = itEdge-> get <0> ();
	const PathPtr_t& pathDE = itEdge-> get <2> ();
	roadmap ()->addEdge (nodeDE, impactNode, pathDE);
	hppDout(info, "connection between q1: " 
		<< displayConfig (*(nodeDE->configuration ()))
		<< "and q2: "
		<< displayConfig (*(impactNode->configuration ())));
      }

      // Insert in roadmap all backward delayed edges (DE)
      for (DelayedEdges_t::const_iterator itEdge = bwdDelayedEdges.begin ();
	   itEdge != bwdDelayedEdges.end (); ++itEdge) {
	const NodePtr_t& nodeDE = itEdge-> get <0> ();
	const PathPtr_t& pathDE = itEdge-> get <2> ();
	roadmap ()->addEdge (impactNode, nodeDE, pathDE);
	hppDout(info, "connection between q1: " 
		<< displayConfig (*(impactNode->configuration ()))
		<< "and q2: "
		<< displayConfig (*(nodeDE->configuration ())));
      }
    }

    void ParabolaPlanner::configurationShooter
    (const ConfigurationShooterPtr_t& shooter)
    {
      configurationShooter_ = shooter;
    }


  } // namespace core
} // namespace hpp
