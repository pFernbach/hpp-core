//crabe8pince
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

#ifndef HPP_CORE_CONFIGURATION_PROJECTION_SHOOTER_HH
# define HPP_CORE_CONFIGURATION_PROJECTION_SHOOTER_HH

# include <sstream>
# include <hpp/model/configuration.hh>
# include <hpp/model/device.hh>
# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>
# include <hpp/core/configuration-shooter.hh>
# include <hpp/core/distance-between-objects.hh>
# include <fcl/distance.h>

namespace hpp {
  namespace core {
    using model::displayConfig;
    /// \addtogroup configuration_sampling
    /// \{

    /// Uniformly sample with bounds of degrees of freedom.
    class HPP_CORE_DLLAPI ConfigurationProjectionShooter :
      public ConfigurationShooter
    {
    public:
      static ConfigurationProjectionShooterPtr_t 
	create (const DevicePtr_t& robot, const Problem &problem,
		const value_type shiftDistance)
      {
	ConfigurationProjectionShooter* ptr =
	  new ConfigurationProjectionShooter (robot, problem, shiftDistance);
	ConfigurationProjectionShooterPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      virtual ConfigurationPtr_t shoot () const
      {
	/* Uniformly sample a configuration */
	JointVector_t jv = robot_->getJointVector ();
	ConfigurationPtr_t config (new Configuration_t (robot_->configSize ()));
	for (JointVector_t::const_iterator itJoint = jv.begin ();
	     itJoint != jv.end (); itJoint++) {
	  std::size_t rank = (*itJoint)->rankInConfiguration ();
	  (*itJoint)->configuration ()->uniformlySample (rank, *config);
	}
	// Shoot extra configuration variables
	size_type extraDim = robot_->extraConfigSpace ().dimension ();
	size_type offset = robot_->configSize () - extraDim;
	for (size_type i=0; i<extraDim; ++i) {
	  value_type lower = robot_->extraConfigSpace ().lower (i);
	  value_type upper = robot_->extraConfigSpace ().upper (i);
	  value_type range = upper - lower;
	  if ((range < 0) ||
	      (range == std::numeric_limits<double>::infinity())) {
	    std::ostringstream oss
	      ("Cannot uniformy sample extra config variable ");
	    oss << i << ". min = " <<lower<< ", max = " << upper << std::endl;
	    throw std::runtime_error (oss.str ());
	  }
	  (*config) [offset + i] = lower + (upper - lower) * rand ()/RAND_MAX;
	}
	/* Project on nearest obstacle and shift away*/
	*config = project (*config);
	hppDout (info, "config: " << displayConfig (*config));
	return config;
      }
	  
      Configuration_t project (const Configuration_t q) const {
	Configuration_t qout = q;
	fcl::Vec3f pi, pj, dir; // fcl nearest points of collision pairs
	value_type minDistance = std::numeric_limits <value_type>::infinity();
	value_type distance = minDistance;

	DistanceBetweenObjectsPtr_t distanceBetweenObjects
	  (problem_.distanceBetweenObjects ()); // problem's distBtwObj not filled yet
	robot_->currentConfiguration (q);
	robot_->computeForwardKinematics ();
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
	const size_type index = robot_->configSize()
	  - robot_->extraConfigSpace ().dimension (); // ecs index
	const size_type ecsDim = robot_->extraConfigSpace ().dimension ();
	
	if (ecsDim == 2) { /* 2D gamma and projection*/
	  qout (0) -= dir [0]; // x part
	  qout (1) -= dir [1]; // y part
	  qout (index) = dir [0]/dir_norm;
	  qout (index+1) = dir [1]/dir_norm;
	}
	else { /* 3D gamma and projection*/
	  qout (0) -= dir [0]; // x part
	  qout (1) -= dir [1]; // y part
	  qout (2) -= dir [2]; // z part
	  qout (index) = dir [0]/dir_norm;
	  qout (index+1) = dir [1]/dir_norm;
	  qout (index+2) = dir [2]/dir_norm;
	}
	hppDout (info, "qout: " << displayConfig (qout));
	
	/* Shift robot away from the contact */
	if (ecsDim == 2) { /* 2D */
	  const value_type gamma = atan2(q (3), q (2)) - M_PI/2;
	  qout (0) -= shiftDistance_*sin(gamma); // x part
	  qout (1) += shiftDistance_*cos(gamma); // y part
	}
	else { /* 3D */
	  qout (0) += shiftDistance_ * q (index); // x part
	  qout (1) += shiftDistance_ * q (index + 1); // y part
	  qout (2) += shiftDistance_ * q (index + 2); // z part
	}
	return qout;
      }
    protected:
      /// Uniformly sample configuration space, then project robot on 
      /// nearest obstacle, at given distance.
      ///
      /// Note that translation joints have to be bounded.
      ConfigurationProjectionShooter (const DevicePtr_t& robot,
				      const Problem &problem,
				      const value_type shiftDistance) :
	problem_ (problem), robot_ (robot),
	shiftDistance_ (shiftDistance)
	{
	}
      void init (const ConfigurationProjectionShooterPtr_t& self)
      {
	ConfigurationShooter::init (self);
	weak_ = self;
      }

    private:
      const Problem& problem_;
      const DevicePtr_t& robot_;
      ConfigurationProjectionShooterWkPtr_t weak_;
      const value_type shiftDistance_;
    }; // class ConfigurationProjectionShooter
    /// \}
  } //   namespace core
} // namespace hpp

#endif // HPP_CORE_CONFIGURATION_PROJECTION_SHOOTER_HH
