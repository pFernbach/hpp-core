// Copyright (c) 2016, LAAS-CNRS
// Authors: Pierre Fernbach (pierre.fernbach@laas.fr)
//
// This file is part of hpp-core.
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
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

# include <hpp/core/steering-method/steering-kinodynamic.hh>

# include <hpp/pinocchio/body.hh>
# include <hpp/pinocchio/joint.hh>
# include <hpp/pinocchio/configuration.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/weighed-distance.hh>
# include <hpp/core/kinodynamic-path.hh>
# include <hpp/core/kinodynamic-oriented-path.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {


      /**
       * @brief sgnenum
       * @param val
       * @return return -1 if val is negative, 1 if val positive, 0 if val == 0
       */
      inline int sgnenum(double val){
        return ((0. < val ) - (val < 0.));
      }

      inline int sgn(double d) {
        return d >= 0.0 ? 1 : -1;
      }

      inline double sgnf(double d) {
        return d >= 0.0 ? 1.0 : -1.0;
      }

      bool belong(double t, interval_t interval){
        return ((t>interval.first) && (t< interval.second));
      }

      double computeMaxRequiredTime(double Tmax, std::vector<interval_t> infIntervals){
        double T;
        bool inInterval = true;
        double minUpperBound = Tmax;

        while(inInterval){
          T = minUpperBound;
          minUpperBound = std::numeric_limits<value_type>::infinity();
          inInterval = false;
          for(size_t i = 0 ; i < infIntervals.size() ; ++i){
            if(belong(T,infIntervals[i]))
              inInterval = true;
            if((infIntervals[i].second < minUpperBound) && (infIntervals[i].second > T))
              minUpperBound = infIntervals[i].second;
          }
        }

        return T;
      }

      PathPtr_t Kinodynamic::impl_compute (ConfigurationIn_t q1,
                                           ConfigurationIn_t q2) const
      {
        double Tmax = 0;
        double T = 0;
        double t0,t1,tv,t2,a1,vLim;
        pinocchio::vector3_t dir(q2[0] - q1[0] ,q2[1] - q1[1] ,q2[2] - q1[2] );
        hppDout(notice,"direction = "<<dir);
        interval_t infeasibleInterval;
        std::vector<interval_t> infIntervalsVector;
        size_type configSize = problem_.robot()->configSize() - problem_.robot()->extraConfigSpace().dimension ();
        // looking for Tmax
        hppDout(notice,"## Looking for Tmax :");
        hppDout(info,"between : "<<pinocchio::displayConfig(q1));
        hppDout(info,"and     : "<<pinocchio::displayConfig(q2));

        // for all joints
        for(int indexConfig = 0 ; indexConfig < 3 ; indexConfig++){ // FIX ME : only work for freeflyer
          size_type indexVel = indexConfig + configSize;
          hppDout(notice,"For joint :"<<problem_.robot()->getJointAtConfigRank(indexConfig)->name());
          if(problem_.robot()->getJointAtConfigRank(indexConfig)->name() != "base_joint_SO3"){
            T = computeMinTime(indexConfig,q1[indexConfig],q2[indexConfig],q1[indexVel],q2[indexVel],&infeasibleInterval);
            infIntervalsVector.push_back(infeasibleInterval);
            if(T > Tmax)
              Tmax = T;
          }else{
            hppDout(notice,"!! Steering method for quaternion not implemented yet.");
          }
        }

        if(Tmax<=std::numeric_limits<double>::epsilon()){
          hppDout(notice,"Steering kinodynamic : no translation mouvement.");
          //TODO : it mean that you must rotate without translation, not implemented yet
          return StraightPath::create(device_.lock(),q1,q2,0.);
        }


        value_type length = computeMaxRequiredTime(Tmax,infIntervalsVector);
        hppDout(notice,"Tmax = "<<Tmax<<"  after infeasible interval : "<<length);
        // create array of times intervals and acceleration values: 
        Configuration_t a1_t(3);
        Configuration_t t0_t(3);
        Configuration_t t1_t(3);
        Configuration_t t2_t(3);
        Configuration_t tv_t(3);
        Configuration_t vLim_t(3);

        
        // compute trajectory with fixed time T found 
        /*for (model::JointVector_t::const_iterator itJoint = jv.begin (); itJoint != jv.end (); itJoint++) {
          size_type indexConfig = (*itJoint)->rankInConfiguration ();
          size_type indexVel = (*itJoint)->rankInVelocity() + configSize;
          hppDout(notice,"For joint "<<(*itJoint)->name());          
          if((*itJoint)->configSize() >= 1){
            fixedTimeTrajectory(Tmax,q1[indexConfig],q2[indexConfig],q1[indexVel],q2[indexVel],&a1,&t1,&tv,&t2);
            a1_t[indexConfig]=a1;
            t1_t[indexConfig]=t1;
            tv_t[indexConfig]=tv;
            t2_t[indexConfig]=t2;     
          }
          
        }// for all joints
        */
        hppDout(info,"compute fixed end-time trajectory for each joint : ");
        for(int indexConfig = 0 ; indexConfig < 3 ; indexConfig++){
          size_type indexVel = indexConfig + configSize;
          hppDout(notice,"For joint :"<<problem_.robot()->getJointAtConfigRank(indexConfig)->name());
          if(problem_.robot()->getJointAtConfigRank(indexConfig)->name() != "base_joint_SO3"){
            fixedTimeTrajectory(indexConfig,length,q1[indexConfig],q2[indexConfig],q1[indexVel],q2[indexVel],&a1,&t0,&t1,&tv,&t2,&vLim);
            a1_t[indexConfig]=a1;
            t0_t[indexConfig]=t0;
            t1_t[indexConfig]=t1;
            tv_t[indexConfig]=tv;
            t2_t[indexConfig]=t2;  
            vLim_t[indexConfig]=vLim;
          }else{
            hppDout(notice,"!! Steering method for quaternion not implemented yet.");
           /* a1_t[indexConfig]=0;
            t1_t[indexConfig]=0;
            tv_t[indexConfig]=0;
            t2_t[indexConfig]=0;  */
          }
        }
        

       KinodynamicPathPtr_t path = KinodynamicPath::create (device_.lock (), q1, q2,length,a1_t,t0_t,t1_t,tv_t,t2_t,vLim_t);
       if(orientedPath_){
         KinodynamicOrientedPathPtr_t orientedPath = KinodynamicOrientedPath::create(path);
         return orientedPath;
       }else{
         return path;
       }

      }
      
      
      
      
      
      Kinodynamic::Kinodynamic (const Problem& problem) :
        SteeringMethod (problem), aMax_(Vector3::Ones(3)),vMax_(Vector3::Ones(3)), device_ (problem.robot ()), weak_ ()
      {
        if(((problem.robot()->extraConfigSpace().dimension())) < 6){
          std::cout<<"Error : you need at least "<<6<<" extra DOF to use this steering method"<<std::endl;
          hppDout(error,"Error : you need at least "<<6<<" extra DOF to use this steering method");
        }

        // get velocity and acceleration bounds from problem :

        aMaxFixed_ = problem_.getParameter(std::string("Kinodynamic/accelerationBound")).floatValue();
        aMax_=Vector3::Ones(3)*aMaxFixed_;
        aMaxFixed_Z_ = problem_.getParameter(std::string("Kinodynamic/verticalAccelerationBound")).floatValue();
        vMax_ = Vector3::Ones(3) *  problem_.getParameter(std::string("Kinodynamic/velocityBound")).floatValue();

        hppDout(info,"#### create steering kinodynamic, vMax = "<<vMax_<< "; aMax_ = "<<aMax_);

        synchronizeVerticalAxis_ = problem_.getParameter(std::string("Kinodynamic/synchronizeVerticalAxis")).boolValue();
        hppDout(notice,"synchronizeVerticalAxis in steering method = "<<synchronizeVerticalAxis_);
        orientedPath_  = problem_.getParameter(std::string("Kinodynamic/forceOrientation")).boolValue();
        hppDout(notice,"oriented path : "<<orientedPath_);
      }
      
      /// Copy constructor
      Kinodynamic::Kinodynamic (const Kinodynamic& other) :
        SteeringMethod (other),aMax_(other.aMax_),vMax_(other.vMax_),synchronizeVerticalAxis_(other.synchronizeVerticalAxis_),orientedPath_(other.orientedPath_),device_ (other.device_)
      {
      }
      
      double Kinodynamic::computeMinTime(int index, double p1, double p2, double v1, double v2, interval_t *infInterval) const{
        hppDout(info,"p1 = "<<p1<<"  p2 = "<<p2<<"   ; v1 = "<<v1<<"    v2 = "<<v2);        
        // compute the sign of each acceleration
        assert(index >= 0 && index < 3 && "index of joint should be between in [0;2]");
        double aMax =std::fabs(aMax_[index]);
        hppDout(notice,"amax used in computeMinTime("<<index<<") = "<<aMax);
        double vMax = vMax_[index];
        double t1,t2,tv;
        int sigma;
        double deltaPacc = 0.5*(v1+v2)*(fabs(v2-v1)/aMax);
        sigma = sgnenum(p2-p1-deltaPacc);  //TODO bug sigma == 0, temp fix ?
        hppDout(notice,"deltaPacc = "<<deltaPacc);
        hppDout(info,"sigma = "<<sigma);
        if(sigma == 0){ // ??? FIXME
          sigma = sgn(p2-p1);
          hppDout(info,"sigma Bis= "<<sigma);
        }
        double a1 = (sigma)*aMax;
        double a2 = -a1;
        double vLim = (sigma) * vMax;
        hppDout(info,"Vlim = "<<vLim<<"   ;  aMax = "<<aMax);
        if((p2-p1) == 0. && (v2-v1)==0. ){  
          hppDout(notice,"No movement in this joints, abort.");
          return 0.;
        }
        // test if two segment trajectory is valid :
        bool twoSegment = false;        

        // solve quadratic equation (cf eq 13 article)
        const double a = a1;
        const double b = 2. * v1;
        const double c = (0.5*(v1+v2)*(v2-v1)/a2) - (p2-p1);

        const double q = -0.5*(b+sgnf(b)*sqrt(b*b-4*a*c));
        hppDout(info, "sign of "<<b<<" is : "<<sgnf(b));
        double x1 = 0;
        double x2 = 0;
        if(a!= 0)
          x1 = q/a;
        if(q!=0)
          x2 = c/q;
        const double x = std::max(x1,x2);
        hppDout(info,"Solve quadratic equation : x1 = "<<x1<<"  ; x2 = "<<x2);
        hppDout(info," x = "<<x);
        hppDout(info,"t1 before vel limit = "<<x);

        hppDout(info,"inf bound on t1 (from t2 > 0) "<<-((v2-v1)/a2));
        double minT1 = std::max(0.,-((v2-v1)/a2));  //lower bound for valid t1 value (cf eq 14)


        if(x >= minT1){
          twoSegment = true;
          t1 = x;
          hppDout(info,"t1 >= minT1");
        } 
        if(twoSegment){ // check if max velocity is respected
          if(std::abs(v1+(t1)*a1) > vMax){
            twoSegment = false;
            hppDout(info,"Doesn't respect max velocity, need 3 segments");
          }
        }
        if(twoSegment){ // compute t2 for two segment trajectory
          tv = 0.;
          t2 = ((v2-v1)/a2) + (t1);// eq 14
        }else{// compute 3 segment trajectory, with constant velocity phase :
          t1 = (vLim - v1)/a1;  //eq 15
          tv = ((v1*v1+v2*v2 - 2*vLim*vLim)/(2*vLim*a1)) + (p2-p1)/vLim ; //eq 16
          t2 = (v2-vLim)/a2;  //eq 17
        }
        if(twoSegment){
          hppDout(notice,"Trajectory with 2 segments");
        }else{
          hppDout(notice,"Trajectory with 3 segments");
        }
        hppDout(notice,"a1 = "<<a1<<"  ;  a2 ="<<a2);
        hppDout(notice,"t = "<<(t1)<<"   ;   "<<(tv)<<"   ;   "<<(t2));
        double T = (t1)+(tv)+(t2);
        hppDout(notice,"T = "<<T);

        // Compute infeasible interval :

        hppDout(info," !!! CHECKING INFEASIBLE INTERVALS : ");

        if(v1 * v2 <= 0.0 || a1 * v1 < 0.0){
          // If minimum-time solution goes through zero-velocity, there is no infeasible time interval, because we can stop and wait at zero-velocity
          infInterval->first=std::numeric_limits<value_type>::infinity();
          infInterval->second=0;
          hppDout(info," ! go through v=0, no infeasible interval");
        }
        else{
          double zeroTime1 = std::abs(v1) / aMax;
          double zeroTime2 = std::abs(v2) / aMax;
          double zeroDistance = zeroTime1 * v1 / 2.0 + zeroTime2 * v2 / 2.0;
          if(std::abs(zeroDistance) < std::abs(p2-p1)) { // no infeasible interval
            infInterval->first = std::numeric_limits<value_type>::infinity();
            infInterval->second = 0;
            hppDout(info," ! not region I");
          }else{ // "region I", infeasible interval exist
            hppDout(info," ! region I, infeasible interval exist");
            a1 = a2;
            a2 = -a2;
            vLim = -vLim;
            // solve eq 13 with new a1/a2 :
            const double ai = a1;
            const double bi = 2. * v1;
            const double ci = (0.5*(v1+v2)*(v2-v1)/a2) - (p2-p1);
            const double deltai = bi*bi - 4.0*ai*ci;
            if(deltai < 0 )
              std::cout<<"Error : determinant of quadratic function negative"<<std::endl;


            const double x1i = (-bi + sqrt(deltai))/(2*ai);
            const double x2i = (-bi - sqrt(deltai))/(2*ai);
            const double xi = std::max(x1i,x2i);
            hppDout(info,"quadratic equation solutions : "<<x1i<<" , "<<x2i);
            // min bound of infeasible interval is given by lesser solution
            t1 = std::min(x1i,x2i);
            infInterval->first = (((v2-v1)/a2) + (t1)) + t1; // eq 14 (T = t2+t1)
            //check if greater solution violate velocity limits :
            minT1 = -minT1;
            if(xi > minT1){
              twoSegment = true;
              t1 = xi;
            }
            if(twoSegment){ // check if max velocity is respected
              if(std::abs(v1+(t1)*a1) > vMax)
                twoSegment = false;
            }
            if(twoSegment){ // compute t2 for two segment trajectory
              tv = 0.;
              t2 = ((v2-v1)/a2) + (t1);// eq 14
            }else{// compute 3 segment trajectory, with constant velocity phase :
              t1 = (vLim - v1)/a1;  //eq 15
              tv = ((v1*v1+v2*v2 - 2*vLim*vLim)/(2*vLim*a1)) + (p2-p1)/vLim ; //eq 16
              t2 = (v2-vLim)/a2;  //eq 17
            }
            infInterval->second = t1+tv+t2;
            hppDout(info,"! INFEASIBLE INTERVAL : ["<<infInterval->first<<" ; "<<infInterval->second<<" ]");
          }
        }

        return T;
      }
      
      void Kinodynamic::fixedTimeTrajectory(int index,double T, double p1, double p2, double v1, double v2, double *a1,double *t0, double *t1, double *tv, double *t2, double *vLim) const{
        hppDout(info,"p1 = "<<p1<<"  p2 = "<<p2<<"   ; v1 = "<<v1<<"    v2 = "<<v2<<" T = "<<T);
        double v12 = v1+v2;
        double v2_1 = v2-v1;
        double p2_1 = p2-p1;
        assert(index >= 0 && index < 3 && "index of joint should be between in [0;2]");
        double vMax = vMax_[index];
        double aMax =std::abs(aMax_[index]);
        hppDout(info,"v12 = "<<v12<<"   ; v21 = "<<v2_1<<"   ; p21 = "<<p2_1);
        int sigma;
        double deltaPacc = 0.5*(v1+v2)*(fabs(v2-v1)/aMax);
        sigma = sgnenum(p2-p1-deltaPacc);
        hppDout(notice,"deltaPacc = "<<deltaPacc);
        hppDout(info,"sigma = "<<sigma);
        if(sigma == 0){
          sigma = sgn(p2-p1);
          hppDout(info,"sigma Bis= "<<sigma);
        }
        
        if(v2_1 == 0 && p2_1 == 0){
          *a1 = 0;
          *t0 = 0;
          *t1 = 0;
          *tv = T;
          *t2 = 0;
          *vLim = 0;
          return;
        }
        if(!synchronizeVerticalAxis_){ // if true : make the trajectory along z axis as short as possible (with the maximal acceleration) and then stay at null acceleration
          if(index == 2 && /*((v1 == 0) ||*/ (v2==0)){ // FIXME : axis z ?
            hppDout(notice, "FIXED TIME TRAJ for axis Z : ");
            assert(index >= 0 && index < 3 && "index of joint should be between in [0;2]");
            *a1 = (sigma)*aMax;
            double a2 = -*a1;
            (*vLim) = (sigma) * vMax;
            hppDout(info,"Vlim = "<<(*vLim)<<"   ;  aMax = "<<aMax);
            // test if two segment trajectory is valid :
            bool twoSegment = false;

            // solve quadratic equation (cf eq 13 article)
            const double a = *a1;
            const double b = 2. * v1;
            const double c = (0.5*(v1+v2)*(v2-v1)/a2) - (p2-p1);

            const double q = -0.5*(b+sgnf(b)*sqrt(b*b-4*a*c));
            double x1 = 0;
            double x2 = 0;
            if(fabs(a)>std::numeric_limits<double>::epsilon()*100.)
              x1 = q/a;
            else
              x1 = sigma*aMax_[index];
            if(fabs(q)>std::numeric_limits<double>::epsilon()*100.)
              x2 = c/q;
            else
              x2 = sigma*aMax_[index];


            hppDout(info, "sign of "<<b<<" is : "<<sgnf(b));

            const double x = std::max(x1,x2);
            hppDout(info,"Solve quadratic equation : x1 = "<<x1<<"  ; x2 = "<<x2);
            hppDout(info," x = "<<x);
            hppDout(info,"t1 before vel limit = "<<x);

            hppDout(info,"inf bound on t1 (from t2 > 0) "<<-((v2-v1)/a2));
            double minT1 = std::max(0.,-((v2-v1)/a2));  //lower bound for valid t1 value (cf eq 14)


            if(x >= minT1){
              twoSegment = true;
              *t1 = x;
              hppDout(info,"t1 >= minT1");
            }
            if(twoSegment){ // check if max velocity is respected
              if(std::abs(v1+(*t1)*(*a1)) > vMax){
                twoSegment = false;
                hppDout(info,"Doesn't respect max velocity, need 3 segments");
              }
            }
            if(twoSegment){ // compute t2 for two segment trajectory
              *tv = 0.;
              *t2 = ((v2-v1)/a2) + (*t1);// eq 14
            }else{// compute 3 segment trajectory, with constant velocity phase :
              *t1 = ((*vLim) - v1)/(*a1);  //eq 15
              *tv = ((v1*v1+v2*v2 - 2*(*vLim)*(*vLim))/(2*(*vLim)*(*a1))) + (p2-p1)/(*vLim) ; //eq 16
              *t2 = (v2-(*vLim))/a2;  //eq 17
            }
            if(twoSegment){
              hppDout(notice,"Trajectory with 2 segments");
            }else{
              hppDout(notice,"Trajectory with 3 segments");
            }
            hppDout(notice,"a1 = "<<(*a1)<<"  ;  a2 ="<<a2);
            if(v1==0)
              *t0 = T - *t1 - *tv - *t2;
            else
              *t0=0;
            hppDout(notice,"t0 = "<<*t0);
            hppDout(notice,"t = "<<(*t1)<<"   ;   "<<(*tv)<<"   ;   "<<(*t2));
            hppDout(notice,"T = "<<T);
            return;
          }else{
            *t0 = 0.;
          }
        }
        *t0 = 0.;



        // quadratic equation (20)
        double a = T*T;
        double b = 2*T*(v12) - 4*p2_1;
        double c = -(v2_1*v2_1);
        const double q = -0.5*(b+sgnf(b)*sqrt(b*b-4*a*c));
        double x1 = 0;
        double x2 = 0;
        if(fabs(a)>std::numeric_limits<double>::epsilon()*100.)
          x1 = q/a;
        else{
          x1 = sigma*aMax_[index];
          hppDout(notice,"a == 0, take x1 = aMax");
        }
        if(fabs(q)>std::numeric_limits<double>::epsilon()*100.)
          x2 = c/q;
        else{
          x2 = sigma*aMax_[index];
          hppDout(notice,"q == 0, take x1 = aMax");
        }
        hppDout(notice,"epsilon = "<<std::numeric_limits<double>::epsilon()*100.);

        hppDout(notice,"a = "<<a<<" ; b = "<<b<<" ; c = "<<c<<" ; q = "<<q);
        hppDout(notice,"x1 = "<<x1<<" ; x2 = "<<x2);
        if(fabs(x1) > fabs(x2))
          *a1 = x1;
        else
          *a1 = x2;
        if(fabs(*a1)>aMax_[index]) // x1 or x2 could be sligtly greater than aMax because of numerical imprecision
          *a1 = aMax_[index] * sgn(*a1);
        double a2 = -(*a1);
        hppDout(notice,"a1 = "<<*a1);
        *t1 = 0.5*((v2_1/(*a1))+T);
        *vLim = sgn((*a1))*vMax;
        hppDout(notice,"vLim = "<<*vLim);
        hppDout(notice,"t1 before velocity limit : "<<*t1);
        if(std::abs(v1+(*t1)*(*a1)) <= vMax){  // two segment trajectory
          hppDout(notice,"Trajectory with 2 segments");
          *t2 = T - (*t1);
          *tv = 0.;
        }else{ // three segment trajectory
          hppDout(notice,"Trajectory with 3 segments");
          // adjust acceleration :
          if(fabs(q)>std::numeric_limits<double>::epsilon()*100.) // otherwise this mean that the solution have only a constant velocity segment, and the following equation will lead to a NaN because (*vLim*T- p2_1) == 0
            *a1 = ((*vLim - v1)*(*vLim - v1) + (*vLim - v2)*(*vLim - v2))/(2*(*vLim*T- p2_1));
          else{
            *a1 =sigma*aMax_[index];
            hppDout(notice,"q == 0, take x1 = aMax");
          }
          if(fabs(*a1)>aMax_[index]) // after this equations a1 could be slightly greater than aMax because of numerical imprecision
            *a1 = aMax_[index] * sgn(*a1);

          a2 = -(*a1);
          // compute new time intervals :
          *t1 = (*vLim - v1)/(*a1);
          *tv = (v1*v1+v2*v2-2*(*vLim)*(*vLim))/(2*(*vLim)*(*a1)) + (p2-p1)/(*vLim) ;
          *t2 = (v2-*vLim)/(a2);
        }
        
        
        hppDout(notice,"a1 = "<<*a1<<"  ;  a2 ="<<a2);
        hppDout(notice,"t = "<<(*t1)<<"   ;   "<<(*tv)<<"   ;   "<<(*t2));
        
      }


      HPP_START_PARAMETER_DECLARATION(Kinodynamic)
      Problem::declareParameter(ParameterDescription (Parameter::FLOAT,
            "Kinodynamic/velocityBound",
            "The maximal magnitude of the velocity along the trajectory. ",
            Parameter(1.)));
      Problem::declareParameter(ParameterDescription (Parameter::FLOAT,
            "Kinodynamic/accelerationBound",
            "The maximal magnitude of the acceleration along the trajectory. ",
            Parameter(10.)));
      Problem::declareParameter(ParameterDescription (Parameter::FLOAT,
            "Kinodynamic/verticalAccelerationBound",
            "The maximal magnitude of the acceleration along the z axis. ",
            Parameter(10.)));
      Problem::declareParameter(ParameterDescription (Parameter::BOOL,
            "Kinodynamic/forceOrientation",
            "If true, the orientation of the root is always aligned with the velocity",
            Parameter(false)));
      Problem::declareParameter(ParameterDescription (Parameter::BOOL,
            "Kinodynamic/synchronizeVerticalAxis",
            "If false, the acceleration along the vertical axis is not synchronized wwith the other axis. This result in a greater vertical acceleration with a longer phase at constant velocity, resulting in a motion with less displacement along the vertical axis",
            Parameter(true)));
      HPP_END_PARAMETER_DECLARATION(Kinodynamic)

      
    } // namespace steeringMethod
  } // namespace core
} // namespace hpp

