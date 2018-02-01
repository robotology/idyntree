/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <kdl/tree.hpp>
#include <sstream>
#include <algorithm>
#include <map>
#include <stack>
#include <iostream>
#include <cassert>

#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>

#include <kdl_codyco/undirectedtree.hpp>
#include <kdl_codyco/utils.hpp>

namespace KDL {
namespace CoDyCo {

    void UndirectedTreeJunction::update_buffers(const double & q) const
    {
        if (q != q_previous) {
            relative_pose_parent_child = joint.pose(q)*f_tip;
            relative_pose_child_parent = relative_pose_parent_child.Inverse();
            //Complicated and inefficient expression, but waiting for testing before simplifyng it (it is copied from working code, so it should work)
            S_child_parent = relative_pose_parent_child.M.Inverse(joint.twist(1.0).RefPoint(joint.pose(q).M * f_tip.p));
            //Compliated and inefficiente expression, but waiting for testing before simplifyng it
            /*
            KDL::Joint inverted_joint;
            KDL::Frame inverted_f_tip;
            JointInvertPolarity(joint,f_tip,inverted_joint,inverted_f_tip);
            S_parent_child = (inverted_joint.pose(q)*inverted_f_tip).M.Inverse(inverted_joint.twist(1.0).RefPoint(inverted_joint.pose(q).M * inverted_f_tip.p));
            */
            S_parent_child = -(relative_pose_parent_child*S_child_parent);
            q_previous = q;
        }
    }

    Frame & UndirectedTreeJunction::pose(const double& q, const bool inverse) const
    {
        update_buffers(q);
        if( !inverse ) {
            return relative_pose_parent_child;
        } else {
            return relative_pose_child_parent;
        }
    }

    Twist & UndirectedTreeJunction::S(const double& q, bool inverse) const
    {
        update_buffers(q);
        if( !inverse ) {
            return S_parent_child;
        } else {
            return S_child_parent;
        }
    }

    Twist UndirectedTreeJunction::vj(const double& q, const double &dq, bool inverse) const
    {
        return S(q,inverse)*dq;
    }

    std::string UndirectedTreeJunction::toString() const
    {
        std::stringstream ss;
        ss << joint_name << " " << q_nr << " "  << joint.getTypeName() << " frame_to_tip " << f_tip;
        return ss.str();
    }

}
}//end of namespace
