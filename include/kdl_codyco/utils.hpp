/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
 
#ifndef KDL_CODYCO_UTILS_HPP
#define KDL_CODYCO_UTILS_HPP

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_codyco/undirectedtree.hpp>
#include <kdl_codyco/momentumjacobian.hpp>
namespace KDL {
namespace CoDyCo {

    /**
     * Compute the total mass of a KDL::Tree object
     * 
     * @param tree the KDL::Tree object
     * @return the mass of the robot, < 0 if something failed
     * 
     */
    double computeMass(const Tree & tree);
    
    /**
     * The position of the Joint with respect to the hook segment is defined with respect
     * to the frame of reference of the parent. This function is used to get the joint with the polarity
     * of the segment inverted.
     * 
     * \todo check the math in this function
     * 
     */
    int JointInvertPolarity(const KDL::Joint & old_joint, const KDL::Frame & old_f_tip, KDL::Joint & new_joint, KDL::Frame & new_f_tip);
    
     /**
       * calculate spatial velocity from momentum: v = inv(I)*h
       * make sure that the spatial momentum h and the inertia are expressed in the same reference frame/point
       * 
       * in case the rigid body inertia as zero mass or a singular inertia tensor, returns a zero twist
       */
     Twist operator/(const Wrench& h, const RigidBodyInertia& I);
     
     bool multiplyInertiaJacobian(const Jacobian& src, const RigidBodyInertia& I, MomentumJacobian& dest);

     bool divideJacobianInertia(const MomentumJacobian& src, const RigidBodyInertia& I, Jacobian& dest);
     
      template<class Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived> & vec)
    {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
    }
    
}
}  



#endif 
