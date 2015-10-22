/*
 * Copyright (C) 2015 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * email: silvio.traversaro@iit.it
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef _IDYNTREE_SIMPLE_LEGGED_ODOMETRY_
#define _IDYNTREE_SIMPLE_LEGGED_ODOMETRY_

#include <iCub/iDynTree/DynTree.h>

namespace iDynTree {
/**
 * This class implements a simple legged odometry scheme for a generic robot.
 *
 * Under the assumption that at least a link of the robot at the time is
 * not moving (no slippage), it computes the estimate of the transform
 * between a inertial/world frame and the robot floating base.
 *
 * The algorithm implemented is the following :
 *
 * 1a) At start (or at reset) the user of the class specifies:
 *       * a frame (world) that should be assumed as the inertial/world frame
 *       * a frame (fixed) that is rigidly attached to a link that is not moving
 *         with respect to the specified inertial frame.
 *     At the start, the world_H_fixed (${}^{world} H_{fixed}$) transfomr between this two specified
 *     frames will be saved.
 * 1b) At this point, the getWorldToFrameTransform(int frame_id) will return the world_H_frame
 *      ( ${}^{world} H_{frame}$ ) transform simply by computing the forward kinematics from the fixed frame
 *     to the frame specified by frame_id : world_H_frame = world_H_fixed * fixed_H_frame(qj)
 *                                          ${}^{world} H_{frame} = {}^{world} H_{fixed} {}^{fixed} H_{frame}(qj)$
 * 2) If the fixed frame changes, we can simply change the frame used as "fixed", and consistently update the
 *      world_H_fixed transform to be equal to world_H_new_fixed =  world_H_old_fixed * old_fixed_H_new_fixed(qj) :
 *      ${}^{world} H_{fixed} = {}^{world} H_{old_fixed} {}^{old_fixed} H_{new_fixed}(qj)$
 * 2b) After the update, the getWorldToFrameTransform(int frame_id) can be obtained as at the point 1b .
 *
 * \note we should update the state used by the odometry by calling the appropritate getDynTree().setAng() method.
 */
    class simpleLeggedOdometry
    {
    private:
        iCub::iDynTree::DynTree * odometry_model;
        int current_fixed_link_id;
        KDL::Frame world_H_fixed;
        
    public:
        simpleLeggedOdometry();
        
        ~simpleLeggedOdometry();
        
        bool init(KDL::CoDyCo::UndirectedTree & undirected_tree,
                  const std::string & initial_world_frame_position,
                  const std::string & initial_fixed_link);
        
        bool init(KDL::CoDyCo::UndirectedTree & undirected_tree,
                  const int initial_world_frame_position_index,
                  const int initial_fixed_link_index);
        
        bool reset(const std::string & initial_world_frame_position,
                   const std::string & initial_fixed_link);
        
        bool reset(const int initial_world_frame_position_index,
                   const int initial_fixed_link_index);
        
        /**
         * Change the link that the odometry assumes to be fixed with the
         * inertial/world frame
         */
        bool changeFixedLink(const std::string & new_fixed_link_name);
        
        /**
         * Change the link that the odometry assumes to be fixed with the
         * inertial/world frame
         */
        bool changeFixedLink(const int & new_fixed_link_id);
        
        /**
         * Get the link currently considered fixed with rispect to the inertial frame.
         * @return the name of the link currently considered fixed.
         */
        std::string getCurrentFixedLink();
        
        /**
         * Get the world_H_frame transform.
         *
         */
        KDL::Frame getWorldFrameTransform(const int frame_index);
        
        /**
         * Set the joint positions, velocities and accelerations
         */
        bool setJointsState(const KDL::JntArray & qj, const KDL::JntArray & dqj, const KDL::JntArray & ddqj);
        
        /**
         * Get iDynTree underlyng object
         *
         */
        const iCub::iDynTree::DynTree & getDynTree();
    };
    
    
} // End namespace iDynTree

#endif // _IDYNTREE_SIMPLE_LEGGED_ODOMETRY_


