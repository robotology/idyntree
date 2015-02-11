/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef KDL_CODYCO_UTILS_HPP
#define KDL_CODYCO_UTILS_HPP

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include "undirectedtree.hpp"
#include "momentumjacobian.hpp"

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
       *
       *
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

    /**
     * Get a std::vector<std::string> where each element is a line of a given file
     *
     * @param line_to_read if it is > 0, it indicates the numer of lines to read, otherwise all the line will be read
     */
    bool stringVectorFromFile(const std::string filename, std::vector<std::string> & strings, int line_to_read=0);

    /**
     * Return true if the base of the input KDL::Tree is a fake base
     * (i.e. a link with only one child, attached to it with a fixed joint and
     *       sharing with it the reference frame)
     * or false if the base link has to be considered a real link.
     *
     * This function is necessary because in KDL::Tree the "base" link cannot have an dynamic inertia (mass, center of mass, ..)
     * so it is custom to use a fake "base" to overcome this problem, but we don't want this "fake"
     * base to be a link in the KDL::CoDyCo::UndirectedTree
     *
     */
    bool isBaseLinkFake(const KDL::Tree & tree);

    /**
     * Convert a spatial acceleration to a classical (6d) acceleration
     *
     * If you have doubt on this conversion, check:
     *    * Featherstone - Rigid Body Dynamics Algorithms - 2008 - Section 2.11
     *    * Featherstone, Roy. "The acceleration vector of a rigid body." The International Journal of Robotics Research 20.11 (2001): 841-846.
     *
     * @param[in] spatial_acc the input spatial acceleration
     * @param[in] velocity the 6D velocity of the rigid body expressed with the same
     *                     orientation and with respect to the same point of the input
     *                     spatial acceleration
     * @param[out] conventional_acc the output conventional acceleration, expressed with the same
     *                              orientation and with respect to the same point of the input
     *                              spatial acceleration
     */
    void spatialToConventionalAcceleration(const KDL::Twist spatial_acc,
                                           const KDL::Twist velocity,
                                                 KDL::Twist & conventional_acc);

     /**
     * Convert a conventional acceleration to a spatial acceleration
     *
     * If you have doubt on this conversion, check:
     *    * Featherstone - Rigid Body Dynamics Algorithms - 2008 - Section 2.11
     *    * Featherstone, Roy. "The acceleration vector of a rigid body." The International Journal of Robotics Research 20.11 (2001): 841-846.
     *
     * @param[in] conventional_acc the conventional 6D acceleration of the rigid body
     * @param[in] velocity the 6D velocity  of the rigid body, expressed with the same
     *                     orientation and with respect to the same point of the input
     *                     conventional acceleration
     * @param[out] spatial_acc the output spatial acceleration, expressed with the same
     *                         orientation and with respect to the same point of the input
     *                         conventional acceleration
     *
     */
    void conventionalToSpatialAcceleration(const KDL::Twist conventional_acc,
                                           const KDL::Twist velocity,
                                                 KDL::Twist & spatial_acc);


}
}



#endif
