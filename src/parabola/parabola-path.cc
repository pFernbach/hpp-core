//
// Copyright (c) 2014 CNRS
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

#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/parabola/parabola-path.hh>

namespace hpp {
  namespace core {
    using model::displayConfig;

    ParabolaPath::ParabolaPath (const DevicePtr_t& device,
				ConfigurationIn_t init,
				ConfigurationIn_t end,
				value_type length,
				vector_t coefs) :
      parent_t (interval_t (0, length), device->configSize (),
		device->numberDof ()), device_ (device), initial_ (init),
      end_ (end), coefficients_ (vector_t(4)), length_ (length)
    {
      assert (device);
      coefficients (coefs);
    }

    ParabolaPath::ParabolaPath (const ParabolaPath& path) :
      parent_t (path), device_ (path.device_), initial_ (path.initial_),
      end_ (path.end_), coefficients_ (path.coefficients_),
      length_ (path.length_)
    {
    }

    bool ParabolaPath::impl_compute (ConfigurationOut_t result,
				     value_type param) const
    {
      if (param == 0) {
	result = initial_;
	return true;
      }
      if (param == length_) {
	result = end_;
	return true;
      }
      
      const size_type nbConfig = device_->configSize();
      const size_type dirDim = device_->extraConfigSpace ().dimension ();
      const value_type u = param/length_;
      const value_type theta = coefficients_(3);
      const value_type x_theta_max = - 0.5 *
	coefficients_ (1) / coefficients_ (0);
      const value_type x_theta_initial = cos(theta)*initial_ (0) +
	sin(theta)*initial_ (1);
      const value_type x_theta_end = cos(theta)*end_ (0) +
	sin(theta)*end_ (1);
      const value_type u_max = (x_theta_max - x_theta_initial)
	/ (x_theta_end - x_theta_initial);
      const bool tanThetaNotDefined = (theta < M_PI/2 + 1e-2 && theta > M_PI/2 - 1e-2) || (theta > -M_PI/2 - 1e-2 && theta < -M_PI/2 + 1e-2);
      
      if (!tanThetaNotDefined) { //theta != +- pi/2
	const value_type tanTheta = tan(theta);
	result (0) = (1 - u)*initial_(0) + u*end_(0);
	result (1) = tanTheta*result (0) -tanTheta*initial_(0) + initial_(1);
	const value_type x_theta = cos(theta)*result (0) +
	  sin(theta)*result (1);
	result (2) = coefficients_(0)*x_theta*x_theta
	  + coefficients_(1)*x_theta + coefficients_(2);
      }
      else { //theta = +- pi/2
	result (0) = initial_ (0);
	result (1) = (1 - u)*initial_(1) + u*end_(1);
	const value_type x_theta = cos(theta)*result (0) +
	  sin(theta)*result (1);
	result (2) = coefficients_(0)*x_theta*x_theta
	  + coefficients_(1)*x_theta + coefficients_(2);
      }

      /* Quaternions interpolation */
      const JointPtr_t SO3joint = device_->getJointByName ("base_joint_SO3");
      const std::size_t rank = SO3joint->rankInConfiguration ();
      const size_type dimSO3 = SO3joint->configSize ();
      SO3joint->configuration ()->interpolate
	(initial_, end_, u, rank, result);

      /* if robot has internal DoF (except freeflyer ones) */
      // translation dimension of freeflyer hardcoded...
      // min value (to reach for u = u_max) hardcoded...
      // manual interpolation since joint not available with index...
      const value_type maxVal = 0; // because here initial_ = end_ ...
      if (nbConfig > dirDim + 3 + dimSO3) {
	for (size_type i = 7; i<nbConfig-dirDim; i++)
	  {
	    if (u <= u_max) {
	      const value_type u_prime = u / u_max;
	      result (i) = (1 - u_prime) * initial_ (i) + u_prime * maxVal;
	    }
	    else {
	      const value_type u_prime = (u - u_max) / (1 - u_max);
	      result (i) = (1 - u_prime) * maxVal + u_prime * end_ (i);
	    }
	  }
      }

      /* Normal vector interpolation
	 result (nbConfig-dirDim) = (1 - u) *
	 initial_(nbConfig-dirDim) + u*end_(nbConfig-dirDim);
	 result (nbConfig-dirDim+1) = (1 - u) *
	 initial_(nbConfig-dirDim+1) + u*end_(nbConfig-dirDim+1);
	 result (nbConfig-dirDim+2) = (1 - u) *
	 initial_(nbConfig-dirDim+2) + u*end_(nbConfig-dirDim+2);*/
      return true;
    }

    PathPtr_t ParabolaPath::extract (const interval_t& subInterval) const
      throw (projection_error)
    {
      bool success;
      //Configuration_t q1 ((*this) (subInterval.first, success)); // straight
      //Configuration_t q2 ((*this) (subInterval.second, success)); // straight
      Configuration_t q1 ((*this) (length_, success));
      Configuration_t q2 ((*this) (0, success));
      PathPtr_t result = ParabolaPath::create (device_, q1, q2, length_,
					       coefficients_);
      return result;
    }

    DevicePtr_t ParabolaPath::device () const
    {
      return device_;
    }
  } //   namespace core
} // namespace hpp

