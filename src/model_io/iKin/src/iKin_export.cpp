/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Silvio Traversaro, CoDyCo project
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Silvio Traversaro */

#include <iDynTree/ModelIO/iKin_export.hpp>

#include <kdl/joint.hpp>
#include <iCub/iKin/iKinFwd.h>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/frames_io.hpp>

using namespace std;

namespace iDynTree{

//Conversion functions
bool YarptoKDL(const yarp::sig::Vector & yarpVector, KDL::Vector & kdlVector)
{
    if( yarpVector.size() != 3 ) return false;
    memcpy(kdlVector.data,yarpVector.data(),3*sizeof(double));
    return true;
}

bool YarptoKDL(const yarp::sig::Matrix & idynMatrix, KDL::Rotation & kdlRotation)
{
    if(idynMatrix.cols() != 3 || idynMatrix.rows() != 3) return false;
    kdlRotation = KDL::Rotation(idynMatrix(0,0),idynMatrix(0,1),idynMatrix(0,2),
                                idynMatrix(1,0),idynMatrix(1,1),idynMatrix(1,2),
                                idynMatrix(2,0),idynMatrix(2,1),idynMatrix(2,2));
    return true;
}

bool YarptoKDL(const yarp::sig::Matrix & idynMatrix, KDL::Frame & kdlFrame)
{
    if( idynMatrix.cols() != 4 || idynMatrix.rows() != 4 ) return false;
    KDL::Rotation kdlRotation;
    KDL::Vector kdlVector;
    yarp::sig::Matrix rot_matrix = idynMatrix.submatrix(0,2,0,2);
    yarp::sig::Vector pos_vector = idynMatrix.subcol(0,3,3);
    YarptoKDL(rot_matrix,kdlRotation);
    YarptoKDL(pos_vector,kdlVector);
    kdlFrame = KDL::Frame(kdlRotation,kdlVector);
    return true;
}

bool KDLtoYarp(const KDL::Vector & kdlVector,yarp::sig::Vector & yarpVector)
{
    if( yarpVector.size() != 3 ) { yarpVector.resize(3); }
    memcpy(yarpVector.data(),kdlVector.data,3*sizeof(double));
    return true;
}

yarp::sig::Vector KDLtoYarp(const KDL::Vector & kdlVector)
{
    yarp::sig::Vector yarpVector;
    KDLtoYarp(kdlVector,yarpVector);
    return yarpVector;
}

bool KDLtoYarp(const KDL::Rotation & kdlRotation, yarp::sig::Matrix & yarpMatrix3_3)
{
    if( yarpMatrix3_3.rows() != 3 || yarpMatrix3_3.cols() != 3 ) { yarpMatrix3_3.resize(3,3); }
    //Both kdl and yarp store the rotation matrix in row major order
    memcpy(yarpMatrix3_3.data(),kdlRotation.data,3*3*sizeof(double));
    return true;
}

bool KDLtoYarp_position(const KDL::Frame & kdlFrame, yarp::sig::Matrix & yarpMatrix4_4 )
{
    yarp::sig::Matrix R(3,3);
    yarp::sig::Vector p(3);

    KDLtoYarp(kdlFrame.M,R);
    KDLtoYarp(kdlFrame.p,p);

    if( yarpMatrix4_4.rows() != 4 || yarpMatrix4_4.cols() != 4 ) { yarpMatrix4_4.resize(4,4); }
    yarpMatrix4_4.zero();

    yarpMatrix4_4.setSubmatrix(R,0,0);
    yarpMatrix4_4.setSubcol(p,0,3);
    yarpMatrix4_4(3,3) = 1;

    return true;
}

yarp::sig::Matrix KDLtoYarp_position(const KDL::Frame & kdlFrame)
{
    yarp::sig::Matrix yarpMatrix4_4(4,4);
    KDLtoYarp_position(kdlFrame,yarpMatrix4_4);
    return yarpMatrix4_4;
}

/**
 * Given two lines, find their closest points (i.e. the points belonging to the common normal)
 *
 * Using the algorithm in http://geomalgorithms.com/a07-_distance.html
 *
 * @return true if the all went well, false it the lines are parallel
 */
bool closestPoints(const KDL::Vector direction_line_A,
                   const KDL::Vector origin_line_A,
                   const KDL::Vector direction_line_B,
                   const KDL::Vector origin_line_B,
                   KDL::Vector & closest_point_line_A,
                   KDL::Vector & closest_point_line_B,
                   double tol = 1e-6
                  )
{
    /*
      Using the notation in : http://geomalgorithms.com/a07-_distance.html
      direction_line_A is u
      origin_line_A is P_0
      direction_line_B is v
      origin_line_b is Q_0
      closest_point_line_A is P_C
      closest_point_line_B is Q_C
    */
    /*
    cerr << "origin_line_A: " << origin_line_A << std::endl;
    cerr << "origin_line_B: " << origin_line_B << std::endl;
    cerr << "direction_line_A: " << direction_line_A << std::endl;
    cerr << "direction_line_B: " << direction_line_B << std::endl;
    */
    KDL::Vector w0 = origin_line_A-origin_line_B;
    double a = dot(direction_line_A,direction_line_A);
    double b = dot(direction_line_A,direction_line_B);
    double c = dot(direction_line_B,direction_line_B);
    double d = dot(direction_line_A,w0);
    double e = dot(direction_line_B,w0);

    double denominator = a*c-b*b;

    // test with fabs because sometimes the numerical zero is computed as -epsilon
    if( fabs(denominator) < tol ) { return false; }

    //Denominator should be nonnegative
    assert(denominator >= 0.0);


    double s_C = (b*e-c*d)/denominator;
    double t_C = (a*e-b*d)/denominator;

    closest_point_line_A = origin_line_A + s_C*direction_line_A;
    closest_point_line_B = origin_line_B + t_C*direction_line_B;

    return true;
}

/**
 * Given two lines, find the dh parameters that describe  the transformation between the two axis
 *                  and return the origin and the x,y axis of the DH reference frame
 *                  (run step 3,4,5,7 of section 3.2.3 of
 *                       http://www.cs.duke.edu/brd/Teaching/Bio/asmb/current/Papers/chap3-forward-kinematics.pdf )
 *                  The input lines and the output origin are expressed in the same frame
 *
 * The _hint variable are necessary when the DH convention allow freedom of choice in choosing dh_direction_axis_x_n or
 *   dh_origin_n, to preserve injectivity.
 */
bool calculateDH(const KDL::Vector direction_axis_z_n_minus_1,
                 const KDL::Vector direction_axis_x_n_minus_1,
                   const KDL::Vector origin_n_minus_1,
                   const KDL::Vector direction_axis_z_n,
                   const KDL::Vector origin_axis_z_n,
                   const KDL::Vector direction_axis_x_n_hint,
                   KDL::Vector & dh_origin_n,
                   KDL::Vector & dh_direction_axis_x_n,
                   KDL::Vector & dh_direction_axis_y_n,
                   double & a_i,
                   double & d_i,
                   double & alpha_i,
                   double & theta_i,
                   double tol = 1e-6,
                   int verbose = 0
                  )
{
    if( verbose > 4 )
    {
    std::cout << "calculateDH called with: " << std::endl;
    std::cout << "direction_axis_z_n_minus_1 " << direction_axis_z_n_minus_1 << std::endl;
    std::cout << "direction_axis_x_n_minus_1 " << direction_axis_x_n_minus_1 << std::endl;
    std::cout << "origin_n_minus_1 " << origin_n_minus_1 << std::endl;
    std::cout << "direction_axis_z_n " << direction_axis_z_n << std::endl;
    std::cout << "origin_axis_z_n " << origin_axis_z_n << std::endl;
    std::cout << "direction_axis_x_n_hint " << direction_axis_x_n_hint << std::endl;
    }

    assert(fabs(direction_axis_z_n_minus_1.Norm()-1) < tol);
    assert(fabs(direction_axis_x_n_minus_1.Norm()-1) < tol);
    assert(fabs(direction_axis_z_n.Norm()-1) < tol);
    assert(fabs(direction_axis_x_n_hint.Norm()-1) < tol);

    //STEP 3
    bool not_parallel;
    KDL::Vector buffer_vector;

    not_parallel = closestPoints(direction_axis_z_n_minus_1,
                                 origin_n_minus_1,
                                 direction_axis_z_n,
                                 origin_axis_z_n,
                                 buffer_vector,
                                 dh_origin_n,
                                 tol);
    if( !not_parallel ) {
        //if parallel, the origin is not specified and we resort to the original one
        dh_origin_n = origin_axis_z_n;
    }

    bool incident = false;
    if( (buffer_vector-dh_origin_n).Norm() < tol ) { incident = true; }

    //STEP 4
    if( !not_parallel ) {

        //If the axis are parallel, the x axis is the common normal of the two axis
        KDL::Vector origin_diff = origin_n_minus_1-dh_origin_n;
        dh_direction_axis_x_n = dot(origin_diff,direction_axis_z_n_minus_1)*direction_axis_z_n_minus_1-origin_diff;

        if( origin_diff.Norm() > tol &&
            fabs(dh_direction_axis_x_n.Norm()) > tol )
        {
            if( verbose > 3 )
            {
                std::cerr << "*** axis_z_n_minus_1 and axis_z_n are parallel ***" << std::endl;
            }
            dh_direction_axis_x_n = dot(origin_diff,direction_axis_z_n_minus_1)*direction_axis_z_n_minus_1-origin_diff;
            if( verbose > 7 )
            {
                std::cerr << "origin_diff : " << origin_diff << std::endl;
                std::cerr << "dh_direction_axis_x_n : " << dh_direction_axis_x_n << std::endl;
            }

            assert( fabs(dh_direction_axis_x_n.Norm()) > tol );

            dh_direction_axis_x_n.Normalize();

            if( verbose > 6 )
            {
                std::cerr << "direction_axis_z_n_minus_1 : " << direction_axis_z_n_minus_1 << std::endl;
                std::cerr << "direction_axis_z_n : " << direction_axis_z_n << std::endl;
                std::cerr << "direction_axis_x_n : " << dh_direction_axis_x_n << std::endl;
            }

            //The positive direction of the axis_x_n is arbitrary, however for dealing with limit case (link where
            // only alpha is different from zero)
            //it is better to have the x_n axis to point in the same direction of the x_n of the original structure description
            //\todo if theta different from zero ?
            //double dh_direction_axis_x_n_sign = dot(dh_direction_axis_x_n, direction_axis_x_n_minus_1);
            double dh_direction_axis_x_n_sign = dot(dh_direction_axis_x_n, direction_axis_x_n_hint);
            dh_direction_axis_x_n = dh_direction_axis_x_n_sign >= 0 ? dh_direction_axis_x_n : -dh_direction_axis_x_n;

            assert((dh_direction_axis_x_n.Norm()-1) < tol);
         }
         else
         {
            if( verbose > 3 )
            {
                std::cerr << "*** axis_z_n_minus_1 and axis_z_n are coincident ***" << std::endl;
            }
            // if the two axis are coincident, the direction of dh_direction_axis_x_n is totally arbitrary
            // as long as it is perpendicular. We will take then the hint provided by : direction_axis_x_n_hint
            // to get the dh_direction_axis_x_n, we will project direction_axis_x_n_hint onto the plane
            // perpendicular to axis_z_n_minus_1 == axis_z_n, and will normalize the resulting vector
            KDL::Vector _direction_axis_z_n = direction_axis_z_n;
            KDL::Vector _direction_axis_x_n_hint = direction_axis_x_n_hint;
            _direction_axis_x_n_hint.Normalize();
            assert( fabs(_direction_axis_x_n_hint.Norm()) > tol );
            _direction_axis_z_n.Normalize();
            dh_direction_axis_x_n = _direction_axis_x_n_hint-dot(_direction_axis_z_n,_direction_axis_x_n_hint)*_direction_axis_z_n;

            assert( fabs(dh_direction_axis_x_n.Norm()) > tol );

            dh_direction_axis_x_n.Normalize();
         }

    } else {

        if( incident ) {
            if(verbose > 3)
                std::cerr << "*** axis_z_n_minus_1 and axis_z_n are incident ***" << std::endl;

            dh_direction_axis_x_n = direction_axis_z_n_minus_1*direction_axis_z_n;
            assert( fabs(dh_direction_axis_x_n.Norm()) > tol );
            dh_direction_axis_x_n.Normalize();

        } else {
            if(verbose > 4)
                std::cerr << "*** axis_z_n_minus_1 and axis_z_n are not incident ***" << std::endl;

            //if the two axis are not incident, the x axis is still the common normal
            dh_direction_axis_x_n  = dh_origin_n-buffer_vector;
            assert( fabs(dh_direction_axis_x_n.Norm()) > tol );

            dh_direction_axis_x_n.Normalize();

        }

        //The positive direction of the axis_x_n is arbitrary, however for dealing with limit case (link where
        // only alpha is different from zero)
        //it is better to have the x_n axis to point in the same direction of the x_n of the original structure description
        //\todo if theta different from zero ?
        //double dh_direction_axis_x_n_sign = dot(dh_direction_axis_x_n, direction_axis_x_n_minus_1);
        double dh_direction_axis_x_n_sign = dot(dh_direction_axis_x_n, direction_axis_x_n_hint);
        dh_direction_axis_x_n = dh_direction_axis_x_n_sign >= 0 ? dh_direction_axis_x_n : -dh_direction_axis_x_n;




    }

    // Once the direction of axis z_n and x_n has been determined,
    //  the direction of axis y_n is simply given by a cross product to
    //  ensure a right handed coordinates system
    dh_direction_axis_y_n = direction_axis_z_n*dh_direction_axis_x_n;


    ////////////////////////////////////////////////////////////////////
    // STEP 5: Computation of DH parameters
    ////////////////////////////////////////////////////////////////////

    //calculation of a_i
      //distance along x_i from O_i to the intersection of the x_i and z_{i-1} axes
    KDL::Vector x_i_z_i_minus_1_intersection_A, x_i_z_i_minus_1_intersection_B;

    if( verbose > 4 )
    {
        std::cout << "dh_direction_axis_x_n      " << dh_direction_axis_x_n << std::endl;
    }

    closestPoints(direction_axis_z_n_minus_1,origin_n_minus_1,
                  dh_direction_axis_x_n,dh_origin_n,
                  x_i_z_i_minus_1_intersection_A,x_i_z_i_minus_1_intersection_B,tol);


    //x_i and z_{i-1} should intersecate
    assert((x_i_z_i_minus_1_intersection_A-x_i_z_i_minus_1_intersection_B).Norm() < tol);


    a_i = -dot(x_i_z_i_minus_1_intersection_B-dh_origin_n,dh_direction_axis_x_n);

    //calculation of d_i
    //distance along z_{i-1} from O_{i-1} to the intersection of the x_i and z_{i-1} axes
    d_i = dot(x_i_z_i_minus_1_intersection_A-origin_n_minus_1,direction_axis_z_n_minus_1);

    //calculation of alpha_i
    //angle between z_{i-1} and z_i measured about x_i
    double cos_alpha_i = dot(direction_axis_z_n_minus_1,direction_axis_z_n);
    assert(((direction_axis_z_n_minus_1*direction_axis_z_n)*dh_direction_axis_x_n).Norm() < tol);
    double sin_alpha_i = dot(direction_axis_z_n_minus_1*direction_axis_z_n,dh_direction_axis_x_n);
    assert( fabs(cos_alpha_i*cos_alpha_i + sin_alpha_i*sin_alpha_i - 1) < tol);

    alpha_i = atan2(sin_alpha_i,cos_alpha_i);

    //calculation of theta_i
    //angle between x_{i-1} and x_i measure about z_{i-1}
    double cos_theta_i = dot(direction_axis_x_n_minus_1,dh_direction_axis_x_n);
    double sin_theta_i = dot(direction_axis_x_n_minus_1*dh_direction_axis_x_n,direction_axis_z_n_minus_1);
    theta_i = atan2(sin_theta_i,cos_theta_i);

    return true;
}


bool iKinLimbFromKDLChain(const KDL::Chain& kdl_chain,
                          iCub::iKin::iKinLimb& iKin_limb,
                          const KDL::JntArray & min,
                          const KDL::JntArray & max,
                          int verbose)
{
    if( verbose > 2)
        std::cout << "iKinLimbFromKDLChain called" << std::endl;


    //Getting Denavit Hartenberg parameters using the algorithm
    //in section 3.2.3 of http://www.cs.duke.edu/brd/Teaching/Bio/asmb/current/Papers/chap3-forward-kinematics.pdf
    //First version, not using fixed joints
    for(unsigned int i=0; i < kdl_chain.getNrOfSegments(); i++ ) {
        KDL::Joint::JointType type = kdl_chain.getSegment(i).getJoint().getType();
        if( type == KDL::Joint::TransX ||
            type == KDL::Joint::TransY ||
            type == KDL::Joint::TransZ ||
            type == KDL::Joint::TransAxis ) {
            std::cerr << "iKinChainFromKDLChain error: Joint type not supported by iKin" << std::endl; return false;
        }
    }

    size_t nj = kdl_chain.getNrOfJoints();

    bool add_joint_limits = true;
    if( min.rows() != nj || max.rows() != nj )
    {
       std::cerr << "iKinChainFromKDLChain error: inconsistent dimension of limits vector" << std::endl; return false;
       add_joint_limits = false;
    }

    //Calculate the ef transform with all joint at zero for future use
    KDL::Frame H_ef_kdl_chain;
    KDL::ChainFkSolverPos_recursive pos_solver(kdl_chain);
    KDL::JntArray q_kdl(nj);
    SetToZero(q_kdl);
    pos_solver.JntToCart(q_kdl,H_ef_kdl_chain);

    // Get all joint axis in the base reference frames
    // The DH parameters are extracted using geometric quantities
    // expressed in the base reference frame for simplicity
    // Reminder: In the DH convention every link is associated to a frame
    //           whose origin is place on the axis of the following joint,
    //           plus the end effector that does not have a following joint,
    //           thus we have NrOfDOFs+1 frames and related axes and origins.
    int nrOfDHFrames = kdl_chain.getNrOfJoints()+1;
    std::vector<KDL::Vector> axis_z_i(nrOfDHFrames);
    std::vector<KDL::Vector> origin_O_i(nrOfDHFrames);
    //Get also the original x axis for when the DH convention is ambiguous
    std::vector<KDL::Vector> axis_x_i_old(nrOfDHFrames);

    int jnt = 0;
    KDL::Frame H = KDL::Frame::Identity();
    for(unsigned int i=0; i < kdl_chain.getNrOfSegments(); i++ )
    {
        // At each for interation, H contains the transform between
        //  the link frame and the base frame

        // Fixed joints (0-DoFs joints) are ignored by the DH convention
        // While prismatic joints are not currently supported by this converter
        KDL::Joint::JointType type = kdl_chain.getSegment(i).getJoint().getType();
        if( type == KDL::Joint::RotX ||
            type == KDL::Joint::RotY ||
            type == KDL::Joint::RotZ ||
            type == KDL::Joint::RotAxis ) {

            axis_z_i[jnt] = H.M*kdl_chain.getSegment(i).getJoint().JointAxis();
            origin_O_i[jnt] = H*kdl_chain.getSegment(i).getJoint().JointOrigin();
            axis_x_i_old[jnt] = H.M.UnitX();

            /*
            KDL::Vector axis_z_i_print = axis_z_i[jnt];
            KDL::Vector origin_O_i_print = origin_O_i[jnt];
            KDL::Vector axis_x_i_old_print = axis_x_i_old[jnt];
            */
            jnt++;

        }

        H = H*kdl_chain.getSegment(i).pose(0.0);

        //std::cout << "H for segment " << i << " : " << H << std::endl;
    }

    assert(jnt == kdl_chain.getNrOfJoints());
    //The last DH frame is in the end effector
    axis_z_i[jnt] = H_ef_kdl_chain.M.UnitZ();
    origin_O_i[jnt] = H_ef_kdl_chain.p;
    axis_x_i_old[jnt] = H_ef_kdl_chain.M.UnitX();


    //Get all DH parameters, computing also the relative x_i and y_i axes
    std::vector<KDL::Vector> axis_x_i(nrOfDHFrames);
    std::vector<KDL::Vector> axis_y_i(nrOfDHFrames);
    std::vector<KDL::Vector> dh_origin_O_i(nrOfDHFrames);

    std::vector<double> a_i(nrOfDHFrames);
    std::vector<double> d_i(nrOfDHFrames);
    std::vector<double> alpha_i(nrOfDHFrames);
    std::vector<double> theta_i(nrOfDHFrames);

    double tol = 1e-5;

    // We will have the H0 transformation to account for the
    // transform between the base frame and the first frame (0)
    // of the DH representation. Hence, we have to choose the
    // first frame for the DH representation. We already have its origin
    // and the z axis, we have then to choose an arbitrary direction for the
    // x and y axis.
    // The strategy for getting this x and y axes is the following:
    //      * if the z axis of the base frame and the 0 DH frame are
    //        parallel, we can assign the x and y axes of the 0 DH frame
    //        to be the same x and y axes of the base frame.
    //      * if the z axis of the base frame and the 0 DH frame are
    //        not parallel, we can find a rotation around an axis that is
    //        transforming the z axis of the base frame to the 0 frame, and
    //        we apply the same transformation to the x and y axes of the base frame
    //        to obtain the x and y axes fo the DH frame.
    //Get arbitrary initial axis (TODO \todo: make a choice that preserve iKin parameters)
    KDL::Vector cross_product = KDL::Vector(0,0,1)*axis_z_i[0];

    bool z_base_and_z_dh_0_are_parallel = ( cross_product.Norm() < tol);

    if( z_base_and_z_dh_0_are_parallel )
    {
        axis_x_i[0] = KDL::Vector(1,0,0);
        axis_y_i[0] = axis_z_i[0]*axis_x_i[0];
    }
    else
    {
        KDL::Vector rot_axis = cross_product;

        assert( fabs(rot_axis.Norm()) > tol );

        rot_axis.Normalize();

        double rot_angle = atan2(dot(rot_axis,cross_product),dot(KDL::Vector(0,0,1),axis_z_i[0]));

        KDL::Rotation rot_base_0 = KDL::Rotation::Rot(rot_axis,rot_angle);

        axis_x_i[0] = rot_base_0.UnitX();
        axis_y_i[0] = rot_base_0.UnitY();

    }


    /*
    cerr << "axis_x_i[0] : " << axis_x_i[0] << endl;
    cerr << "axis_y_i[0] : " << axis_y_i[0] << endl;
    cerr << "axis_z_i[0] : " << axis_z_i[0] << endl;

    cerr << "dot(axis_x_i[0],axis_z_i[0]) : " << dot(axis_x_i[0],axis_z_i[0]) << endl;
    cerr << "dot(axis_y_i[0],axis_z_i[0]) : " << dot(axis_y_i[0],axis_z_i[0]) << endl;
    cerr << "cross_product.Norm(): " << cross_product.Norm() << endl;
    cerr << "axis_x_i[0]*axis_y_i[0] : " << axis_x_i[0]*axis_y_i[0] << endl;
    */
    assert(dot(axis_x_i[0],axis_z_i[0]) < tol);
    assert(dot(axis_y_i[0],axis_z_i[0]) < tol);
    assert((axis_x_i[0]*axis_y_i[0]-axis_z_i[0]).Norm() < tol);


    dh_origin_O_i[0] = origin_O_i[0];



    for(size_t i=0; i < nj; i++ ) {
        //std::cout << "i: " << i << std::endl;
        assert(fabs(axis_z_i[i].Norm()-1) < tol);
        assert(fabs(axis_z_i[i+1].Norm()-1) < tol);
        calculateDH(axis_z_i[i],
                    axis_x_i[i],
                    dh_origin_O_i[i],
                    axis_z_i[i+1],
                    origin_O_i[i+1],
                    axis_x_i_old[i+1],
                    dh_origin_O_i[i+1],
                    axis_x_i[i+1],
                    axis_y_i[i+1],
                    a_i[i+1],
                    d_i[i+1],
                    alpha_i[i+1],
                    theta_i[i+1],
                    tol,
                    verbose);
    }

    KDL::Frame H0_kdl; //rigid roto-translation matrix from the root reference frame to the 0th frames
    KDL::Frame HN_kdl; //the rigid roto-translation matrix from the Nth frame to the end-effector.

    H0_kdl = KDL::Frame(KDL::Rotation(axis_x_i[0],axis_y_i[0],axis_z_i[0]),origin_O_i[0]);

    //iCub::iKin::iKinChain converted_iKin_chain;

    //Create converted_iKin_chain
    iCub::iKin::iKinChain & iKin_chain = *(iKin_limb.asChain());

    iKin_chain.clear();

    bool ok = iKin_chain.setH0(KDLtoYarp_position(H0_kdl));
    assert(ok);
    (void) ok; 
    
    yarp::sig::Matrix R(3,3);
    KDLtoYarp(H0_kdl.M,R);

    // Check that the rototranslation matrix is a rototranslation
    //cerr << " yarp::math::det(R) " <<  yarp::math::det(R) << endl;
    assert(fabs(yarp::math::det(R)-1) < tol);

    if( verbose > 2 )
        std::cout << "iKinLink H0 " << H0_kdl << endl;


    for(size_t i=0; i < nj; i++ )
    {
        iCub::iKin::iKinLink * p_new_link;
        if( !add_joint_limits )
        {
            p_new_link = new iCub::iKin::iKinLink(a_i[i+1],d_i[i+1],alpha_i[i+1],theta_i[i+1]);
        }
        else
        {
            p_new_link = new iCub::iKin::iKinLink(a_i[i+1],d_i[i+1],alpha_i[i+1],theta_i[i+1],min(i),max(i));
        }

        iKin_chain.pushLink(*p_new_link);

        if( verbose > 2 )
            std::cout << "iKinLink added " << a_i[i+1] << " " << d_i[i+1] << " " << alpha_i[i+1] << " " << theta_i[i+1] << endl;
    }

    //Calculate the base pose with iKinChain (with HN = Identity) and the KDL::Chain . The difference is the desired HN
    KDL::Frame H_ef_iKinChain;

    iKin_chain.setAllConstraints(false);
    for(size_t i=0; i < nj; i++ ) { iKin_chain.releaseLink(i); }
    for(size_t i=0; i < nj; i++ ) { iKin_chain.setAng(i,0.0); }
    yarp::sig::Matrix H_ef_iKinChain_yarp = iKin_chain.getH();
    YarptoKDL(H_ef_iKinChain_yarp,H_ef_iKinChain);

    HN_kdl = H_ef_iKinChain.Inverse()*H_ef_kdl_chain;

    bool ret = iKin_chain.setHN(KDLtoYarp_position(HN_kdl));

    R.zero();
    KDLtoYarp(HN_kdl.M,R);

    // Check that the rototranslation matrix is a rototranslation
    assert(fabs(yarp::math::det(R)-1) < tol);

    assert(ret);
    (void) ret;

    if( verbose > 2 )
        std::cout << "iKinLink HN " << HN_kdl << endl;


    //iKin_chain = converted_iKin_chain;

    for(size_t i=0; i < nj; i++ ) { iKin_chain.releaseLink(i); }

    if(verbose >2)
        std::cout << "iKinChainFromKDLChain: closing routine" << std::endl;


    return true;
}


}

