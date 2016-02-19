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

#ifndef HPP_CORE_PARABOLA_PLANNER_HH
# define HPP_CORE_PARABOLA_PLANNER_HH

# include <hpp/core/path-planner.hh>

namespace hpp {
  namespace core {
    /// \addtogroup path_planning
    /// \{

    /// Generic implementation of RRT algorithm
    class HPP_CORE_DLLAPI ParabolaPlanner : public PathPlanner
    {
    public:
      /// Return shared pointer to new object.
      static ParabolaPlannerPtr_t createWithRoadmap
	(const Problem& problem, const RoadmapPtr_t& roadmap);

      /// Return shared pointer to new object.
      static ParabolaPlannerPtr_t create (const Problem& problem);

      /// One step of extension.
      virtual void oneStep ();

      /// Set configuration shooter.
      void configurationShooter (const ConfigurationShooterPtr_t& shooter);

    protected:
      /// Constructor
      ParabolaPlanner (const Problem& problem, const RoadmapPtr_t& roadmap);

      /// Constructor with roadmap
      ParabolaPlanner (const Problem& problem);

      /// Store weak pointer to itself
      void init (const ParabolaPlannerWkPtr_t& weak);

    private:
      ConfigurationShooterPtr_t configurationShooter_;
      ParabolaPlannerWkPtr_t weakPtr_;
    };
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PARABOLA_PLANNER_HH
