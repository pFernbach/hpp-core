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

#ifndef HPP_CORE_STEERING_METHOD_PARABOLA_HH
# define HPP_CORE_STEERING_METHOD_PARABOLA_HH

# include <hpp/util/debug.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/steering-method.hh>
# include <hpp/core/straight-path.hh>
# include <hpp/core/weighed-distance.hh>

namespace hpp {
  namespace core {
    /// \addtogroup steering_method
    /// \{

    /// Steering method that creates StraightPath instances
    ///
    class HPP_CORE_DLLAPI SteeringMethodParabola : public SteeringMethod
    {
    public:
      /// Create instance and return shared pointer
      static SteeringMethodParabolaPtr_t create (const ProblemPtr_t& problem)
      {
	SteeringMethodParabola* ptr = new SteeringMethodParabola (problem);
	SteeringMethodParabolaPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Copy instance and return shared pointer
      static SteeringMethodParabolaPtr_t createCopy
	(const SteeringMethodParabolaPtr_t& other)
      {
	SteeringMethodParabola* ptr = new SteeringMethodParabola (*other);
	SteeringMethodParabolaPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Copy instance and return shared pointer
      virtual SteeringMethodPtr_t copy () const
      {
	return createCopy (weak_.lock ());
      }

      /// create a path between two configurations
      virtual PathPtr_t impl_compute (ConfigurationIn_t q1,
				      ConfigurationIn_t q2) const;
    protected:
      /// Constructor with problem
      /// Robot and weighed distance are created from problem
      SteeringMethodParabola (const ProblemPtr_t& problem);

      /// Copy constructor
      SteeringMethodParabola (const SteeringMethodParabola& other) :
	SteeringMethod (other), device_ (other.device_),
	distance_ (other.distance_), weak_ (), g_(other.g_),
	V0max_ (other.V0max_), Vimpmax_ (other.Vimpmax_),mu_ (other.mu_),
	Dalpha_ (other.Dalpha_), workspaceDim_ (other.workspaceDim_)
	{
	}

      /// Store weak pointer to itself
      void init (SteeringMethodParabolaWkPtr_t weak)
      {
	SteeringMethod::init (weak);
	weak_ = weak;
      }
    private:
      /// 2D impl_compute
      PathPtr_t compute_2D_path (ConfigurationIn_t q1,
				 ConfigurationIn_t q2) const;

      /// 3D impl_compute
      PathPtr_t compute_3D_path (ConfigurationIn_t q1,
				 ConfigurationIn_t q2) const;

      /// Compute second constraint: V0 <= V0max
      /// return false if constraint can never be respected.
      /// fill alpha_lim_plus/minus angles limiting initial angle
      /// to respect the constraint.
      bool second_constraint (const value_type& X, const value_type& Y,
			      value_type *alpha_lim_plus,
			      value_type *alpha_lim_minus) const;

      /// Compute third constraint : landing in the friction cone
      /// return false if constraint can never be respected.
      /// fill alpha_imp_sup/inf angles limiting initial angle
      /// to respect the constraint.
      bool third_constraint (bool fail, const value_type& X,
			     const value_type& Y,
			     const value_type alpha_imp_min,
			     const value_type alpha_imp_max,
			     value_type *alpha_imp_sup,
			     value_type *alpha_imp_inf,
			     const value_type n2_angle) const;
		
      /// Compute fiveth constraint: compute intersection between coneS
      /// and plane_theta. If not empty, compute the two lines and the angle
      /// between them = 2*delta.
      /// Equations have been obtained using Matlab.
      bool fiveth_constraint (const ConfigurationIn_t q,
			      const value_type theta,
			      const int number,
			      value_type *delta) const;

      /// Compute sixth constraint: V_imp <= V_imp_max
      /// return false if constraint can never be respected.
      /// fill alpha_imp_plus/minus angles limiting initial angle
      /// to respect the constraint.
      bool sixth_constraint (const value_type& X, const value_type& Y,
			     value_type *alpha_imp_plus,
			     value_type *alpha_imp_minus) const;

      /// Get the length of the path by numerical integration (Simpson method)
      /// Length is computed only when the path is created
      virtual value_type computeLength (const ConfigurationIn_t q1,
					const ConfigurationIn_t q2,
					const vector_t coefs) const;

      /// Function equivalent to sqrt( 1 + f'(x)^2 ) in 2D
      /// Function equivalent to sqrt( 1 + y0_dot/x0_dot + fz'(x)^2 ) in 3D
      value_type lengthFunction (const value_type x,const vector_t coefs) const;

      ProblemPtr_t problem_;
      DeviceWkPtr_t device_;
      WeighedDistancePtr_t distance_;
      SteeringMethodParabolaWkPtr_t weak_;
      value_type g_; // gravity constant
      mutable value_type V0max_; // maximal initial velocity
      mutable value_type Vimpmax_; // maximal landing velocity
      mutable value_type mu_; // friction coefficient
      value_type Dalpha_; // alpha increment
      mutable bool workspaceDim_; // true for 3D, false for 2D
    }; // SteeringMethodParabola
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_STEERING_METHOD_PARABOLA_HH
