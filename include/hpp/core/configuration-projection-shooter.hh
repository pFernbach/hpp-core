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
# include <hpp/model/device.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/configuration-shooter.hh>

namespace hpp {
  namespace core {
    /// \addtogroup configuration_sampling
    /// \{

    /// Uniformly sample with bounds of degrees of freedom.
    class HPP_CORE_DLLAPI ConfigurationProjectionShooter :
      public ConfigurationShooter
    {
    public:
      static ConfigurationProjectionShooterPtr_t 
	create (const DevicePtr_t& robot, const Problem &problem)
      {
	ConfigurationProjectionShooter* ptr =
	  new ConfigurationProjectionShooter (robot, problem);
	ConfigurationProjectionShooterPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      /// Process successively an uniform sample and a projection
      virtual ConfigurationPtr_t shoot () const;

      /// Uniformly sample a configuration
      Configuration_t uniformlySample () const;

      /// Project the given configuration on the nearest obstacle,
      /// then shift it to avoid contact
      Configuration_t project (const Configuration_t q) const;
      
    protected:
      /// Uniformly sample configuration space, then project robot on 
      /// nearest obstacle, at given distance.
      ///
      /// Note that translation joints have to be bounded.
      ConfigurationProjectionShooter (const DevicePtr_t& robot,
				      const Problem &problem) :
	problem_ (problem), robot_ (robot),
	shiftDistance_ (problem.shiftDistance_)
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
