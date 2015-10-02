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

# include <hpp/util/debug.hh>
# include <hpp/model/configuration.hh>
# include <hpp/model/device.hh>
# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>
# include <hpp/core/contact-configuration-shooter.hh>
# include <hpp/model/collision-object.hh>
# include <hpp/fcl/collision_object.h>
# include <hpp/fcl/BVH/BVH_model.h>
# include <hpp/core/collision-validation.hh>

namespace hpp {
  using namespace fcl;
  namespace core {
    using model::displayConfig;
    typedef fcl::BVHModel<OBBRSS> BVHModelOB;
    typedef boost::shared_ptr<const BVHModelOB> BVHModelOBConst_Ptr_t;

    BVHModelOBConst_Ptr_t GetModel (const fcl::CollisionObjectConstPtr_t object)
    {
      assert (object->collisionGeometry ()->getNodeType () == BV_OBBRSS);
      const BVHModelOBConst_Ptr_t model
	= boost::static_pointer_cast<const BVHModelOB>
	(object->collisionGeometry ());
      assert (model->getModelType () == BVH_MODEL_TRIANGLES);
      return model;
    }

    double TriangleArea (ContactConfigurationShooter::TrianglePoints& tri)
    {
      double a, b, c;
      a = (tri.p1 - tri.p2).norm();
      b = (tri.p2 - tri.p3).norm();
      c = (tri.p3 - tri.p1).norm();
      double s = 0.5 * (a + b + c);
      return sqrt(s * (s-a) * (s-b) * (s-c));
    }

    void SetConfigTranslation(ConfigurationPtr_t config,
			      const Vec3f& translation)
    {
      for(int i =0; i<3; ++i)
        {
	  (*config)(i)=translation[i];
        }
    }

    void Translate(ConfigurationPtr_t config, const Vec3f& translation)
    {
      for(int i =0; i<3; ++i)
        {
	  (*config)(i)+=translation[i];
        }
    }

    void SampleRotation(ConfigurationPtr_t config, JointVector_t& jv)
    {
      JointPtr_t joint = jv[1];
      std::size_t rank = joint->rankInConfiguration ();
      joint->configuration ()->uniformlySample (rank, *config);
    }

    void ContactConfigurationShooter::InitWeightedTriangles
    (const model::ObjectVector_t& geometries)
    {
      double sum = 0;
      hppDout (info, "geometries.size (): " << geometries.size ());
      for(model::ObjectVector_t::const_iterator objit = geometries.begin();
	  objit != geometries.end(); ++objit)
	{
	  const  fcl::CollisionObjectPtr_t& colObj = (*objit)->fcl();
	  BVHModelOBConst_Ptr_t model =  GetModel(colObj); // TODO NOT TRIANGLES
	  hppDout (info, "model->num_tris: " << model->num_tris);
	  for(int i =0; i < model->num_tris; ++i)
	    {
	      TrianglePoints tri;
	      Triangle fcltri = model->tri_indices[i];
	      tri.p1 = colObj->getRotation() *
		model->vertices[fcltri[0]] + colObj->getTranslation();
	      tri.p2 = colObj->getRotation() *
		model->vertices[fcltri[1]] + colObj->getTranslation();
	      tri.p3 = colObj->getRotation() *
		model->vertices[fcltri[2]] + colObj->getTranslation();;
	      double weight = TriangleArea(tri);
	      sum += weight;
	      weights_.push_back(weight);
	      // TODO COMPUTE NORMALS
	      fcl::Vec3f normal = (tri.p3 - tri.p1).cross(tri.p2 - tri.p1);
	      normal.normalize();
	      triangles_.push_back(std::make_pair(normal,tri));
	    }
	  double previousWeight = 0;
	  for(std::vector<double>::iterator wit = weights_.begin();
	      wit != weights_.end(); ++wit)
	    {
	      previousWeight += (*wit) / sum;
	      (*wit) = previousWeight;
	    }
        }
    }

    const ContactConfigurationShooter::T_TriangleNormal
    &ContactConfigurationShooter::RandomPointIntriangle () const
    {
      return triangles_ [rand() % triangles_.size()];
    }

    const ContactConfigurationShooter::T_TriangleNormal&
    ContactConfigurationShooter::WeightedTriangle() const
    {
      double r = ((double) rand() / (RAND_MAX));
      std::vector<T_TriangleNormal>::const_iterator trit = triangles_.begin();
      for(std::vector<double>::const_iterator wit = weights_.begin();
          wit != weights_.end();
          ++wit, ++trit)
	{
          if(*wit <= r) return *trit;
	}
      return triangles_[triangles_.size()-1]; // not supposed to happen
    }
     
    Configuration_t ContactConfigurationShooter::setOrientation
    (const Configuration_t& q) const {
      Configuration_t qtest = q;
      const JointPtr_t jointSO3 = robot_->getJointVector () [1];
      const size_type indexSO3 = jointSO3->rankInConfiguration ();
      const size_type index = robot_->configSize ()
	- robot_->extraConfigSpace ().dimension ();

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

    ConfigurationPtr_t ContactConfigurationShooter::shoot () const
    {
      JointVector_t jv = robot_->getJointVector ();
      ConfigurationPtr_t config (new Configuration_t (robot_->configSize ()));
      std::size_t limit = shootLimit_;

      bool found(false);
      //do {
      while(limit >0 && !found)
	{
	  // pick one triangle randomly
	  const T_TriangleNormal* sampled(0);
	  double r = ((double) rand() / (RAND_MAX));
	  if(r > 0.3)
	    sampled = &RandomPointIntriangle();
	  else
	    sampled = &WeightedTriangle();
	  const TrianglePoints& tri = sampled->second;
	  //stackoverflow.com/questions/4778147/sample-random-point-in-triangle
	  double r1, r2;
	  r1 = ((double) rand() / (RAND_MAX));
	  r2 = ((double) rand() / (RAND_MAX));
	  Vec3f p = (1 - sqrt(r1)) * tri.p1 + (sqrt(r1) * (1 - r2)) * tri.p2
	    + (sqrt(r1) * r2) * tri.p3;

	  //set configuration position to sampled point
	  SetConfigTranslation(config, p);
	  SampleRotation(config, jv);
	  //hppDout (info, "config (after move): " << displayConfig (*config));
	  // rotate and translate randomly until valid configuration found or
	  // no obstacle is reachable
	  CollisionValidationReport report;
	  std::size_t limitDis = displacementLimit_;
	  Vec3f lastDirection(1,0,0);
	  while(!found && limitDis >0)
	    {
	      if(validator_->validate(*config, report))
		{
		  found = true;
		}
	      else // move out of collision
		{
		  // retrieve Contact information
		  //lastDirection = -report.result.getContact(0).normal;
		  // mouve out by penetration depth
		  // v0 move away from normal
		  //get normal from collision tri
		  lastDirection = triangles_
		    [report.result.getContact(0).b2].first;
		  //hppDout (info, "lastDirection: " << lastDirection);
		  Translate(config, -lastDirection * 
			    (std::abs(report.result.getContact (0).
				      penetration_depth) +0.03));
		  //hppDout (info, "config (after translate): " << displayConfig (*config));
		  limitDis--;
		}
	    }
      
	  //hppDout (info, "surface normal: " << -lastDirection);
	  // Set extra configuration variables as the surface DIRECTION
	  const size_type extraDim = robot_->extraConfigSpace ().dimension ();
	  const size_type offset = robot_->configSize () - extraDim;
	  for (size_type i=0; i<extraDim; ++i)
	    {
	      (*config) [offset + i] = -lastDirection [i];
	    }
	  limit--;
	}
      //if (!found) hppDout (info, "no config found by shooter");
      //hppDout (info, "config (before setOrien): " << displayConfig (*config));

      *config = setOrientation (*config);
      //hppDout (info, "config: " << displayConfig (*config));
      //}
      // orientation may break validity
      //while (!validator_->validate(*config, report));
      return config;
    }

  } //   namespace core
} // namespace hpp
