//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
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

#ifndef HPP_CORE_CONTACT_CONFIGURATION_SHOOTER_HH
# define HPP_CORE_CONTACT_CONFIGURATION_SHOOTER_HH

# include <hpp/util/debug.hh>
# include <sstream>
# include <hpp/model/device.hh>
# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>
# include <hpp/core/configuration-shooter.hh>
# include <hpp/core/collision-validation.hh>
# include <hpp/core/problem.hh>

namespace hpp {
  namespace core {
    /// \addtogroup configuration_sampling
    /// \{

    /// Uniformly sample with bounds of degrees of freedom.
    class HPP_CORE_DLLAPI ContactConfigurationShooter :
      public ConfigurationShooter
    {
    public:
      struct TrianglePoints
      {
        fcl::Vec3f p1, p2, p3;
      };
      typedef std::pair<fcl::Vec3f, TrianglePoints> T_TriangleNormal;

      static ContactConfigurationShooterPtr_t
	create (const DevicePtr_t& robot, const Problem &problem)
      {
	ContactConfigurationShooter* ptr
	  = new ContactConfigurationShooter (robot, problem);
	ContactConfigurationShooterPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      /// Sample a random configuration at a surface contact
      /// Inspired by RbPrm sampler
      virtual ConfigurationPtr_t shoot () const;
 
    protected:
      /// Constructor, based on the RbPrmShooter one
      /// Taking the problem as argument to gather obstacles geometries
      ContactConfigurationShooter (const DevicePtr_t& robot,
				   const Problem &problem)
	: robot_ (robot)
	, shootLimit_ (1000)
	, displacementLimit_ (100)
	, validator_ (CollisionValidation::create (robot_))
	{
	  weights_.clear ();
	  triangles_.clear ();
	  const ObjectVector_t& geometries = problem.collisionObstacles ();
	  for(ObjectVector_t::const_iterator cit = geometries.begin ();
	      cit != geometries.end(); ++cit)
	    {
	      validator_->addObstacle(*cit);
	    }
	  this->InitWeightedTriangles(geometries);
	}

      void init (const ContactConfigurationShooterPtr_t& self)
      {
	ConfigurationShooter::init (self);
	weak_ = self;
      }

    private:
      void InitWeightedTriangles(const model::ObjectVector_t &geometries);
      const T_TriangleNormal& RandomPointIntriangle() const;
      const T_TriangleNormal& WeightedTriangle() const;

      const DevicePtr_t& robot_;
      ContactConfigurationShooterWkPtr_t weak_;
      const std::size_t shootLimit_;
      const std::size_t displacementLimit_;
      CollisionValidationPtr_t validator_;
      std::vector<double> weights_;
      std::vector<T_TriangleNormal> triangles_;
    }; // class ContactConfigurationShooter
    /// \}
  } //   namespace core
} // namespace hpp

#endif // HPP_CORE_CONTACT_CONFIGURATION_SHOOTER_HH
