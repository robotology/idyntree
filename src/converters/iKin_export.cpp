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

#include "kdl_format_io/iKin_export.hpp"'
#include <kdl/joint.hpp>
#include <ace/config-posix.h>


using namespace std;

namespace kdl_format_io{

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
    KDL::Vector w0 = origin_line_A-origin_line_B;
    double a = direction_line_A*direction_line_A;
    double b = direction_line_A*direction_line_B;
    double c = direction_line_B*direction_line_B;
    double d = direction_line_A*w0;
    double e = direction_line_B*w0;
    
    double denominator = a*c-b*b;
    
    //Denominator should be nonnegative
    assert(denominator >= 0.0);
    
    if( denominator < tol ) { return false; }
    
    double s_C = (b*e-c*d)/denominator;
    double t_C = (a*e-b*d)/denominator;
    
    closest_point_line_A = origin_line_A + s_C*direction_line_A;
    closest_point_line_B = origin_line_B + t_C*direction_line_B;
    
    return true;
}

bool iKinChainFromKDLChain(const KDL::Chain& kdl_chain, iCub::iKin::iKinChain& iKin_chain)
{
    iCub::iKin::iKinChain converted_iKin_chain;
    
    //Getting Denavit Hartenberg parameters using the algorithm 
    //in section 3.2.3 of http://www.cs.duke.edu/brd/Teaching/Bio/asmb/current/Papers/chap3-forward-kinematics.pdf 
    //First version, not using fixed joints
    for(int i=0; i < kdl_chain.getNrOfSegments(); i++ ) {
        KDL::Joint::JointType type = kdl_chain.getSegment(i).getJoint().getType();
        if( type == KDL::Joint::TransX ||
            type == KDL::Joint::TransY ||
            type == KDL::Joint::TransZ ||
            type == KDL::Joint::TransAxis ||
            type == KDL::Joint::None ) {
            std::cerr << "iKinChainFromKDLChain error: Joint type not supported by iKin" << std::endl; return false;
        }
    }
    
    //STEP 3 
    //The initial frame is defined implicitly in the chain, the next frame for DH
    //is defined by the axis of the first rotational joint
    //locate new_origin following Step 3 of algorithm
    KDL::Vector new_origin, dummy;
    KDL::Vector z_axis(0,0,1);
    KDL::Vector origin(0,0,0);
    
    KDL::Vector joint_axis =  kdl_chain.getSegment(0).getJoint().JointAxis();
    KDL::Vector joint_origin = kdl_chain.getSegment(0).getJoint().JointOrigin();
    
    joint_axis.Normalize();
    
    bool not_parallel;
    not_parallel = closestPoints(z_axis,
                   origin,
                   joint_axis,
                   joint_origin,
                   dummy,
                   new_origin,
                   1e-4
                 );
    
    if( !not_parallel ) {
        //if the axis are parallel use the old origin of the joint frame
        new_origin = joint_origin;
    }
    
    //STEP 4
    KDL::Vector new_x_axis;
    if( !not_parallel ) {
        //If the axis are parallel, the x axis is the common normal of the two axis
        KDL::Vector origin_diff = origin-joint_origin;
        new_x_axis = origin_diff-(origin_diff*joint_axis)*joint_axis;
        new_x_axis.Normalize();
    } else {
        if( incident ) { 
            
        } else {
            //if the two axis are not incident, the x axis is still the common normal
            new_x_axis = dummy-new_origin;
            new_x_axis.Normalize();
        }
    }
    
    
    
    //STEP 5 
    
    for(int i=0; i < kdl_chain.getNrOfSegments(); i++ ) {
        
    }
    
    
    iKin_chain = converted_iKin_chain;
    return true;
}


}

