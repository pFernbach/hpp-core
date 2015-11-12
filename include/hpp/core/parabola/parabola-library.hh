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

#ifndef HPP_CORE_PARABOLA_LIBRARY_HH
# define HPP_CORE_PARABOLA_LIBRARY_HH

# include <sstream>
# include <hpp/util/debug.hh>
# include <hpp/model/configuration.hh>
# include <hpp/model/device.hh>

namespace hpp {
  namespace core {
    using model::displayConfig;

    /// Arrange robot orientation according to the surface normal direction
    /// So that the robot is "on" the surface, rotated
    inline Configuration_t setOrientation
    (const DevicePtr_t& robot, const Configuration_t& q) {
      Configuration_t qtest = q;
      const JointPtr_t jointSO3 = robot->getJointVector () [1];
      const size_type indexSO3 = jointSO3->rankInConfiguration ();
      const size_type index = robot->configSize ()
	- robot->extraConfigSpace ().dimension ();

      const value_type nx = q [index];
      const value_type ny = q [index + 1];
      const value_type nz = q [index + 2];
      //const value_type theta = atan2 (2*qw*qz - 2*qx*qy, 1-2*qy*qy-2*qz*qz);
      const value_type theta = q [index + 3];
      hppDout (info, "theta: " << theta);

      // most general case (see Matlab script)
      const value_type x12= nz/sqrt((1+tan(theta)*tan(theta))
				    *nz*nz+(nx+ny*tan(theta))*(nx+ny*tan(theta)));
      const value_type y12 = x12*tan(theta);
      const value_type z12 = -x12*(nx+ny*tan(theta))/nz;
      const value_type zx = nz*x12-nx*z12;
      const value_type yz = ny*z12-nz*y12;
      const value_type xy = nx*y12-ny*x12;
      fcl::Matrix3f A; // A: rotation matrix expressing R_r in R_0
      A (0,0) = x12; A (0,1) = yz; A (0,2) = nx;
      A (1,0) = y12; A (1,1) = zx; A (1,2) = ny;
      A (2,0) = z12; A (2,1) = xy; A (2,2) = nz;
      hppDout (info, "A: " << A);
      hppDout (info, "A.determinant (): " << A.determinant ());

      fcl::Quaternion3f quat;
      quat.fromRotation (A);
      hppDout (info, "quat: " << quat);
	
      qtest [indexSO3] = quat [0];
      qtest [indexSO3 + 1] = quat [1];
      qtest [indexSO3 + 2] = quat [2];
      qtest [indexSO3 + 3] = quat [3];
      hppDout (info, "qtest: " << displayConfig (qtest));
      return qtest;
    }
  } //   namespace core
} // namespace hpp

#endif // HPP_CORE_PARABOLA_LIBRARY_HH
