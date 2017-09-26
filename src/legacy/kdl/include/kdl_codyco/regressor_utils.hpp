/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
     
#ifndef _KDL_CODYCO_REGRESSOR_UTILS_HPP
#define _KDL_CODYCO_REGRESSOR_UTILS_HPP

#ifdef __DEPRECATED
  #warning <regressor_utils.hpp> is deprecated.
#endif

#include <Eigen/Core>

#include <kdl/rotationalinertia.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/jntarray.hpp>




namespace KDL {
namespace CoDyCo {    
    
    Eigen::Matrix<double,10,1> Vectorize(const KDL::RigidBodyInertia & rbd_inertia);

    
    Eigen::Matrix<double,6,1> vech(const KDL::RotationalInertia & rot_inertia);
    
    KDL::RigidBodyInertia deVectorize(const Eigen::Matrix<double,10,1> & vec);
    KDL::RotationalInertia devech(const Eigen::Matrix<double,6,1> & vec);
    
    /**
     * 
     * crossProductMatrix(v)*u = v*u
     * 
     */
    Eigen::Matrix3d crossProductMatrix(const KDL::Vector & v);
    
    Eigen::Matrix<double, 6, 6> spatialCrossProductTwistMatrix(const KDL::Twist & v);
    
    Eigen::Matrix<double, 6, 6> spatialCrossProductWrenchMatrix(const KDL::Twist & v);
    
    Eigen::Matrix<double, 6, 6> TwistTransformationMatrix(const KDL::Frame & frame);
    
    Eigen::Matrix<double, 6, 6> WrenchTransformationMatrix(const KDL::Frame & frame);


    Eigen::Matrix<double, 3, 6> rotationalMomentumRegressor(const KDL::Vector & w);


    /**
     * Get the momentum regressor for a given spatial twist.
     *
     * The momentum regressor is the 6x10 matrix such that:
     *      momentumRegressor(v)*Vectorize(I) == I*v
     * 
     * @
     */
    Eigen::Matrix<double, 6, 10> momentumRegressor(const KDL::Twist & v);
    
    /**
     * Get the net wrench regressor for a given spatial twist and spatial acceleration
     * 
     * The momentum regressor is 6x10 matrix such that:
     *      netWrenchRegressor(v,a) = I*a + v*I*v
     * 
     */
    Eigen::Matrix<double, 6, 10> netWrenchRegressor(const KDL::Twist & v, const KDL::Twist & a) ;    
    
    /**
     * Get a Eigen 6 element vector from a KDL::Twist (0:2 linear velocity, 3:5 angular velocity)
     * 
     */
    Eigen::Matrix<double, 6, 1> toEigen(const KDL::Twist & v);
    
    /**
     * Get an Eigen 6 element vector from a KDL::Wrench (0:2 linear force, 3:5 torque)
     * 
     */
    Eigen::Matrix<double, 6, 1> toEigen(const KDL::Wrench & v);
    
    Eigen::Matrix<double, 6, 6> toEigen(const KDL::RigidBodyInertia & I);
    
    Eigen::VectorXd toEigen(const KDL::Wrench & f, const KDL::JntArray & tau);
    
    Eigen::VectorXd toEigen(const KDL::Twist & v, const KDL::JntArray & dq);


    KDL::Wrench toKDLWrench(const Eigen::Matrix<double, 6, 1> & in);
    
    KDL::Twist toKDLTwist(const Eigen::Matrix<double, 6, 1> & in);
}
}
             
#endif
