/*
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
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

#include "iDynTree/Estimation/simpleLeggedOdometry.h"

#include "kdl/frames_io.hpp"

namespace iDynTree
{

simpleLeggedOdometry::simpleLeggedOdometry():
odometry_model(0),
current_fixed_link_id(-1),
world_H_fixed()
{
    
}

simpleLeggedOdometry::~simpleLeggedOdometry()
{
    if( odometry_model )
    {
        delete odometry_model;
        odometry_model=0;
    }
}

bool simpleLeggedOdometry::init(KDL::CoDyCo::UndirectedTree & undirected_tree,
                                const std::string& initial_world_frame_position,
                                const std::string& initial_fixed_link)
{
    int initial_world_frame_position_index = undirected_tree.getLink(initial_world_frame_position)->getLinkIndex();
    int initial_fixed_link_index = undirected_tree.getLink(initial_fixed_link)->getLinkIndex();;
    if( initial_fixed_link_index < 0 ||
       initial_world_frame_position_index < 0 )
    {
        return false;
    }
    
    return init(undirected_tree,initial_world_frame_position_index,initial_fixed_link_index);
}
    
bool simpleLeggedOdometry::init(KDL::CoDyCo::UndirectedTree & undirected_tree,
                                const std::string& initial_world_frame_position,
                                const std::string& initial_fixed_link,
                                KDL::Vector initial_world_offset)
{
    int initial_world_frame_position_index = undirected_tree.getLink(initial_world_frame_position)->getLinkIndex();
    int initial_fixed_link_index = undirected_tree.getLink(initial_fixed_link)->getLinkIndex();;
    if( initial_fixed_link_index < 0 ||
       initial_world_frame_position_index < 0 )
    {
        return false;
    }
    
    return init(undirected_tree,initial_world_frame_position_index,initial_fixed_link_index,initial_world_offset);
}


bool simpleLeggedOdometry::init(KDL::CoDyCo::UndirectedTree & undirected_tree,
                                const int initial_world_frame_position_index,
                                const int initial_fixed_link_index)
{
    if( odometry_model )
    {
        delete odometry_model;
        odometry_model=0;
    }
    
    odometry_model = new iCub::iDynTree::DynTree(undirected_tree.getTree(),undirected_tree.getSerialization());
    bool ok = reset(initial_world_frame_position_index,initial_fixed_link_index);
    return ok;
}

bool simpleLeggedOdometry::init(KDL::CoDyCo::UndirectedTree & undirected_tree,
                                const int initial_world_frame_position_index,
                                const int initial_fixed_link_index,
                                KDL::Vector initial_world_offset)
{
    if( odometry_model )
    {
        delete odometry_model;
        odometry_model=0;
    }
    
    odometry_model = new iCub::iDynTree::DynTree(undirected_tree.getTree(),undirected_tree.getSerialization());
    bool ok = reset(initial_world_frame_position_index,initial_fixed_link_index,initial_world_offset);
    return ok;
}

    
bool simpleLeggedOdometry::reset(const std::string& initial_world_frame_position, const std::string& initial_fixed_link)
{
    int initial_world_frame_position_index = odometry_model->getLinkIndex(initial_world_frame_position);
    int initial_fixed_link_index = odometry_model->getLinkIndex(initial_fixed_link);
    if( initial_fixed_link_index < 0 ||
       initial_world_frame_position_index < 0 )
    {
        return false;
    }
    
    return reset(initial_world_frame_position_index,initial_fixed_link_index);
}

bool simpleLeggedOdometry::reset(const int initial_world_frame_position_index, const int initial_fixed_link_index)
{
    current_fixed_link_id = initial_fixed_link_index;
    world_H_fixed = odometry_model->getPositionKDL(initial_world_frame_position_index,initial_fixed_link_index);
    return true;
}
    
bool simpleLeggedOdometry::reset(const int initial_world_frame_position_index, const int initial_fixed_link_index, KDL::Vector initial_world_offset)
{
    current_fixed_link_id = initial_fixed_link_index;
    world_H_fixed = odometry_model->getPositionKDL(initial_world_frame_position_index,initial_fixed_link_index, initial_world_offset);
    return true;
}

bool simpleLeggedOdometry::changeFixedLink(const std::string& new_fixed_link_name)
{
    int new_fixed_link_id = odometry_model->getLinkIndex(new_fixed_link_name);
    
    if( new_fixed_link_id < 0 )
    {
        return false;
    }
    
    return changeFixedLink(new_fixed_link_id);
}


bool simpleLeggedOdometry::changeFixedLink(const int& new_fixed_link_id)
{
    int old_fixed_link_id = this->current_fixed_link_id;
    KDL::Frame world_H_old_fixed = this->world_H_fixed;
    KDL::Frame old_fixed_H_new_fixed = odometry_model->getPositionKDL(old_fixed_link_id,new_fixed_link_id);
    this->world_H_fixed = world_H_old_fixed*old_fixed_H_new_fixed;
    this->current_fixed_link_id = new_fixed_link_id;
    return true;
}

//bool simpleLeggedOdometry::changeFixedLink(const std::string &new_fixed_link_name, const KDL::Frame world_H_old_fixed)
//{
//    int new_fixed_link_id = odometry_model->getLinkIndex(new_fixed_link_name);
//    
//    if ( new_fixed_link_id < 0 )
//    {
//        return false;
//    }
//    
//    return changeFixedLink(new_fixed_link_id, world_H_old_fixed);
//}
//    
//bool simpleLeggedOdometry::changeFixedLink(const int &new_fixed_link_id, const KDL::Frame world_H_old_fixed)
//{
//    int old_fixed_link_id = this->current_fixed_link_id;
//    KDL::Frame old_fixed_H_new_fixed = odometry_model->getPositionKDL(old_fixed_link_id,new_fixed_link_id);
//    this->world_H_fixed = world_H_old_fixed*old_fixed_H_new_fixed;
//    this->current_fixed_link_id = new_fixed_link_id;
//    return true;
//}
    
bool simpleLeggedOdometry::changeFixedFoot()
{
    if ( !std::strcmp(this->getCurrentFixedLink().c_str(), std::string("l_sole").c_str())
         || !std::strcmp(this->getCurrentFixedLink().c_str(), std::string("r_sole").c_str()) )
        {
            if ( !std::strcmp(this->getCurrentFixedLink().c_str(), std::string("l_sole").c_str()) )
            {
                changeFixedLink(std::string("r_sole"));
            }
            if ( !std::strcmp(this->getCurrentFixedLink().c_str(), std::string("r_sole").c_str()) )
            {
                changeFixedLink(std::string("l_sole"));
            }
        }
    else {
        yError("[simpleLeggedOdometry::changeFixedFoot] The current fixed link is not a foot. Used simpleLeggedOdometry::changeFixedLink() instead...");
        return false;
    }
    return true;
}
    

std::string simpleLeggedOdometry::getCurrentFixedLink()
{
    std::string ret_string;
    odometry_model->getLinkName(this->current_fixed_link_id,ret_string);
    return ret_string;
}

KDL::Frame simpleLeggedOdometry::getWorldFrameTransform(const int frame_index)
{
    KDL::Frame fixed_H_frame = odometry_model->getPositionKDL(this->current_fixed_link_id,frame_index);
    return this->world_H_fixed*fixed_H_frame;
}

bool simpleLeggedOdometry::setJointsState(const KDL::JntArray& qj,
                                          const KDL::JntArray& dqj,
                                          const KDL::JntArray& ddqj)
{
    if( qj.rows() != odometry_model->getNrOfDOFs()  ||
       dqj.rows() != odometry_model->getNrOfDOFs()  ||
       ddqj.rows() != odometry_model->getNrOfDOFs() )
    {
        return false;
    }
    
    bool ok = true ;
    
    ok = ok && odometry_model->setAngKDL(qj);
    ok = ok && odometry_model->setDAngKDL(dqj);
    ok = ok && odometry_model->setD2AngKDL(ddqj);
    
    //Update also the floating base position, given this new joint positions
    KDL::Frame world_H_base = this->getWorldFrameTransform(odometry_model->getFloatingBaseLink());
    ok = ok && odometry_model->setWorldBasePoseKDL(world_H_base);
    
    return ok;
}

const iCub::iDynTree::DynTree & simpleLeggedOdometry::getDynTree()
{
    return *odometry_model;
}


} // iDynTree namespace









