//
// Copyright (c) 2015 CNRS
// Authors: Mylene Campana
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
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
#include <hpp/core/distance-between-objects.hh>
#include <fcl/distance.h>
#include <boost/tuple/tuple.hpp>
#include <hpp/core/contact-configuration-shooter.hh>

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
      PathPlanner (problem),
      workspaceDim_ (false)
    {
      configurationShooter (BasicConfigurationShooter::create
			    (problem.robot ()));
    }

    ParabolaPlanner::ParabolaPlanner (const Problem& problem,
				      const RoadmapPtr_t& roadmap) :
      PathPlanner (problem, roadmap),
      workspaceDim_ (false)
    {
      if (problem.collisionObstacles ().size () > 0) {
	hppDout (info, "create contact config shooter");
	configurationShooter (ContactConfigurationShooter::create
			      (problem.robot (), problem));
      }
      else
	configurationShooter (BasicConfigurationShooter::create
			      (problem.robot ()));
    }

    void ParabolaPlanner::init (const ParabolaPlannerWkPtr_t& weak)
    {
      PathPlanner::init (weak);
      weakPtr_ = weak;
    }

    Configuration_t ParabolaPlanner::project (const Configuration_t& q)
    {
      Configuration_t q_proj = q;
      DevicePtr_t robot (problem ().robot ());
      fcl::Vec3f pi, pj, dir; // fcl nearest points of collision pairs
      value_type minDistance = std::numeric_limits <value_type>::infinity();
      value_type distance = minDistance;

      DistanceBetweenObjectsPtr_t distanceBetweenObjects
	(problem ().distanceBetweenObjects ());
      robot->currentConfiguration (q);
      robot->computeForwardKinematics ();
      distanceBetweenObjects->computeDistances (); // only outers !
      const model::DistanceResults_t& dr =
	distanceBetweenObjects->distanceResults ();

      for (model::DistanceResults_t::const_iterator itDistance = 
	     dr.begin (); itDistance != dr.end (); itDistance++) {
	distance = itDistance->distance ();
	if (distance < minDistance){
	  minDistance = distance;
	  pi = itDistance->closestPointInner (); // point Body
	  pj = itDistance->closestPointOuter (); // point Obst
	  dir = pi - pj; // obstacle normale direction
	}
      }
      hppDout (info, "minDistance: " << minDistance);
      hppDout (info, "pi: " << pi);
      hppDout (info, "pj: " << pj);
      hppDout (info, "dir: " << dir);

      const value_type dir_norm = sqrt (dir [0]*dir [0] + dir [1]*dir [1]
					+ dir [2]*dir [2]);
      const size_type index = robot->configSize()
	- robot->extraConfigSpace ().dimension (); // ecs index
      if (!workspaceDim_) { /* 2D gamma and projection*/
	q_proj (0) -= dir [0]; // x part
	q_proj (1) -= dir [1]; // y part
	q_proj (index) = dir [0]/dir_norm;
	q_proj (index+1) = dir [1]/dir_norm;
      }
      else { /* 3D gamma and projection*/
	q_proj (0) -= dir [0]; // x part
	q_proj (1) -= dir [1]; // y part
	q_proj (2) -= dir [2]; // z part
	q_proj (index) = dir [0]/dir_norm;
	q_proj (index+1) = dir [1]/dir_norm;
	q_proj (index+2) = dir [2]/dir_norm;
      }
      hppDout (info, "q_proj: " << displayConfig (q_proj));
      return q_proj;
    }
	
    Configuration_t ParabolaPlanner::contactShift(const Configuration_t& q)
      const {
      Configuration_t q_shift = q;
      const value_type dist = 0.01; // shift distance
      if (!workspaceDim_) { /* 2D */
	const value_type gamma = atan2(q (3), q (2)) - M_PI/2;
	q_shift (0) -= dist*sin(gamma); // x part
	q_shift (1) += dist*cos(gamma); // y part
      }
      else { /* 3D */
	const size_type index = problem ().robot ()->configSize()
	  - problem ().robot ()->extraConfigSpace ().dimension ();
	q_shift (0) += dist * q (index); // x part
	q_shift (1) += dist * q (index + 1); // y part
	q_shift (2) += dist * q (index + 2); // z part
      }
      return q_shift;
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
      bool fwdEdgeExists, bwdEdgeExists, validConfig;
      DelayedEdge_t fwdDelayedEdge, bwdDelayedEdge;
      DelayedEdges_t fwdDelayedEdges, bwdDelayedEdges;
      /*const size_type index = robot->configSize()
	- robot->extraConfigSpace ().dimension ();*/

      /* Define dimension: 2D or 3D */
      std::string name = robot->getJointVector () [0]->name ();
      hppDout (info, "first joint name = " << name); // debug
      if (name == "base_joint_xyz") // 3D
	workspaceDim_ = true; // 3D (2D by default)
      
      // shoot a valid random configuration
      // and try to project this configuration at the contact of an obstacle
      ConfigurationPtr_t qrand;
      Configuration_t q_tmp (robot->configSize ());
      do {
	validConfig = false;
	qrand = configurationShooter_->shoot ();
	if (configValidations->validate (*qrand)) {
	  //q_tmp = project (*qrand);
	  q_tmp = *qrand;
	  validConfig = true;
	  /*if(!workspaceDim_)
	    validConfig = q_tmp (index + 1) >= 0;
	    else
	    validConfig = q_tmp (index + 2) >= 0;
	  */
	}
      } while (!validConfig);
      hppDout (info, "qrand: " << displayConfig (*qrand));

      /* Set to zero DoF other than translations and dir
	 NOT ADAPTED TO QUATERNION
	 if (robot->configSize() > robot->extraConfigSpace ().dimension ()*2) { 
	 for (size_type i = robot->extraConfigSpace ().dimension ();
	 i < index; i++) {
	 q_tmp (i) = 0;
	 }
	 }*/
      hppDout (info, "q_tmp: " << displayConfig (q_tmp));
      //ConfigurationPtr_t q_proj ( new Configuration_t (contactShift(q_tmp)));
      ConfigurationPtr_t q_proj ( new Configuration_t (q_tmp));
      hppDout (info, "q_proj: " << displayConfig (*q_proj));
      

      // Add q_proj as a new node: here for the parabola, as the impact node
      core::NodePtr_t impactNode = roadmap ()->addNode (q_proj);

      // try to connect the random configuration to each connected component
      // of the roadmap.
      for (ConnectedComponents_t::const_iterator itcc =
	     roadmap ()->connectedComponents ().begin ();
	   itcc != roadmap ()->connectedComponents ().end (); ++itcc) {
	ConnectedComponentPtr_t cc = *itcc;
	fwdEdgeExists = false;
	bwdEdgeExists = false;
	// except its own connected component of course
	if (cc != impactNode->connectedComponent ()) {
	  value_type length = std::numeric_limits <value_type>::infinity ();

	  // iteration on each node of the current connected-component
	  for (Nodes_t::const_iterator n_it = cc->nodes ().begin (); 
	       n_it != cc->nodes ().end (); ++n_it){
	    ConfigurationPtr_t qCC = (*n_it)->configuration ();

	    // Create forward local path from qCC to q_proj
	    localPath = (*sm) (*qCC, *q_proj);

	    // validate forward local path
	    if (localPath &&
		pathValidation->validate (localPath, false, validPart)) {
	      hppDout (info, "forward path from SM is valid");
	      // Save node from current cc that leads to shortest path
	      if (localPath->length () < length) {
		fwdEdgeExists = true;
		length = localPath->length ();
		// Save forward delayed edge from cc with shorter path
		fwdDelayedEdge = DelayedEdge_t (*n_it, q_proj, localPath);
		hppDout (info, "forward delayed edge is saved");
	      }
	    }

	    // Create backward local path from q_proj to qCC
	    localPath = (*sm) (*q_proj, *qCC);

	    // validate backward local path
	    if (localPath &&
		pathValidation->validate (localPath, false, validPart)) {
	      hppDout (info, "backward path from SM is valid");
	      // Save node from current cc that leads to shortest path
	      if (localPath->length () < length) {
		bwdEdgeExists = true;
		length = localPath->length ();
		// Save backward delayed edge from cc with shorter path
		// keeping n_it* information for later edge adding...
		bwdDelayedEdge = DelayedEdge_t (*n_it, q_proj, localPath);
		hppDout (info, "backward delayed edge is saved");
	      }
	    }

	  }
	  if (fwdEdgeExists) // avoid adding a null delayed-edge ...
	    fwdDelayedEdges.push_back (fwdDelayedEdge); // shortest path from cc
	  if (bwdEdgeExists) // avoid adding a null delayed-edge ...
	    bwdDelayedEdges.push_back (bwdDelayedEdge); // shortest path from cc
	  
	}// for nodes in cc
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
