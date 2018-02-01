/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef _IDYNTREE_SIMPLE_LEGGED_ODOMETRY_
#define _IDYNTREE_SIMPLE_LEGGED_ODOMETRY_

#include <iCub/iDynTree/DynTree.h>

namespace iDynTree {
/**
 * \ingroup iDynTreeEstimation
 *
 * A simple legged odometry for a legged robot.
 *
 *
 * Under the assumption that at least a link of the robot at the time is
 * not moving (no slippage), it computes the estimate of the transform
 * between a inertial/world frame and the robot floating base.
 *
 * The algorithm implemented is the following :
 *
 * -# During initialization (or at reset) the user of the class specifies (through init()):
 *       * a frame (world) that should be assumed as the inertial/world frame (`initial_world_frame_position` or `initial_world_frame_position_index`)
 *       * a frame (fixed) that is rigidly attached to a link that is not moving (`initial_fixed_link` or `initial_fixed_link_index`)
 *         with respect to the specified inertial frame.
 *     At the beginning, the `world_H_fixed` (\f${}^{world} H_{fixed}\f$) transform between these two specified
 *     frames will be saved.
 *
 * -# At this point, the `getWorldFrameTransform(int frame_id)` will return the `world_H_frame`
 *      ( \f${}^{world} H_{frame}\f$ ) transform simply by computing the forward kinematics from the fixed frame
 *     to the frame specified by `frame_id` : `world_H_frame = world_H_fixed * fixed_H_frame(qj)`, i.e.
 *                                          \f${}^{world} H_{frame} = {}^{world} H_{fixed} \cdot {}^{fixed} H_{frame}(q_j)\f$
 *
 * -# If the fixed frame changes, we can simply change the frame used as "fixed" (changeFixedLink()), and consistently update the
 *      `world_H_fixed` transform to be equal to `world_H_new_fixed =  world_H_old_fixed * old_fixed_H_new_fixed(qj)`, i.e.
 *      \f${}^{world} H_{new\_fixed} = {}^{world} H_{old\_fixed} \cdot {}^{old\_fixed} H_{new\_fixed}(q_j)\f$
 *
 * -# After the update, the `getWorldFrameTransform(int frame_id)` can be obtained as in point 1b .
 *
 * \note we should update the state used by the odometry by calling the appropriate `getDynTree().setAng()` method.
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
                  const std::string & initial_world_frame_position,
                  const std::string & initial_fixed_link,
                  KDL::Vector initial_world_offset);

        bool init(KDL::CoDyCo::UndirectedTree & undirected_tree,
                  const int initial_world_frame_position_index,
                  const int initial_fixed_link_index);

        bool init(KDL::CoDyCo::UndirectedTree & undirected_tree,
                  const int initial_world_frame_position_index,
                  const int initial_fixed_link_index,
                  KDL::Vector initial_world_offset);

        bool reset(const std::string & initial_world_frame_position,
                   const std::string & initial_fixed_link);

        bool reset(const int initial_world_frame_position_index,
                   const int initial_fixed_link_index);

        bool reset(const int initial_world_frame_position_index,
                   const int initial_fixed_link_index,
                   KDL::Vector initial_world_offset);


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

//        /**
//         * Change the link that the odometry assumes to be fixed along with
//         * world_H_old_fixed if you want world to remain where initially specified.
//         */
//        bool changeFixedLink(const std::string & new_fixed_link_name, const KDL::Frame world_H_old_fixed);
//
//        /**
//         * Change the link that the odometry assumes to be fixed along with
//         * world_H_old_fixed if you want world to remain where initially specified.
//         */
//        bool changeFixedLink(const int & new_fixed_link_id, const KDL::Frame world_H_old_fixed);

        /**
         *  This method changes fixed link from l_sole to r_foot and vicerversa.
         *
         *  @return True when successful, false otherwise.
         */
        bool changeFixedFoot();

        /**
         * Get the link currently considered fixed with respect to the inertial frame.
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


