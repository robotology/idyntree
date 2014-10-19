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

#include "kdl_format_io/urdf_export.hpp"
#include <urdf_model/model.h>
#include <iostream>
#include <urdf_parser/urdf_parser.h>
#include <tinyxml.h>
#include <kdl/tree.hpp>
#include <urdf_model/model.h>
#include <kdl/joint.hpp>
#include "kdl_format_io/config.h"

using namespace std;

namespace kdl_format_io {

// construct vector
urdf::Vector3 toUrdf(const KDL::Vector & v)
{
    return urdf::Vector3(v.x(), v.y(), v.z());
}

// construct rotation
urdf::Rotation toUrdf(const KDL::Rotation & r)
{
    double x,y,z,w;
    r.GetQuaternion(x,y,z,w);
    return urdf::Rotation(x,y,z,w);
}

// construct pose
urdf::Pose toUrdf(const KDL::Frame & p)
{
    urdf::Pose ret;
    ret.rotation = toUrdf(p.M);
    ret.position = toUrdf(p.p);
    return ret;
}


KDL::Frame getH_new_old(KDL::Joint jnt, KDL::Frame frameToTip)
{
    KDL::Frame H_new_old;
    if( (jnt.JointOrigin()-frameToTip.p).Norm() < 1e-12 || jnt.getType() == KDL::Joint::None ) {
        //No need of changing link frame
        H_new_old = KDL::Frame::Identity();
    } else {
        std::cerr << "[WARN] the reference frame of link connected to joint " << jnt.getName()  << "  has to be shifted to comply to URDF constraints" << std::endl;
        //H_new_old = (KDL::Frame(jnt.JointOrigin())*KDL::Frame(frameToTip.M)).Inverse()*frameToTip;
        H_new_old = KDL::Frame(frameToTip.M.Inverse()*(frameToTip.p-jnt.JointOrigin()));
    }
    return H_new_old;
}

KDL::Frame getH_new_old(KDL::Segment seg)
{
    KDL::Joint jnt =  seg.getJoint();
    KDL::Frame frameToTip = seg.getFrameToTip();
    return getH_new_old(jnt,frameToTip);
}


// construct joint
/**
 *
 * @param jnt the KDL::Joint to convert (axis and origin expressed in the
 *          predecessor frame of reference, as by KDL convention)
 * @param frameToTip the predecessor/successor frame transformation
 * @param H_new_old_predecessor in the case the predecessor frame is being
 *          modified to comply to URDF constraints (frame origin on the joint axis)
 *          this matrix the transformation from the old frame to the new frame (H_new_old)
 * @param H_new_old_successor  in the case the successor frame is being
 *          modified to comply to URDF constraints (frame origin on the joint axis)
 *          this matrix the transformation from the old frame to the new frame (H_new_old)
 */
urdf::Joint toUrdf(const KDL::Joint & jnt, const KDL::Frame & frameToTip, const KDL::Frame & H_new_old_predecessor, KDL::Frame & H_new_old_successor)
{
    //URDF constaints the successor link frame origin to lay on the axis
    //of the joint ( see : http://www.ros.org/wiki/urdf/XML/joint )
    //Then if the JointOrigin of the KDL joint is not zero, it is necessary
    //to move the link frame (then it is necessary to change also the spatial inertia)
    //and the definition of the childrens of the successor frame
    urdf::Joint ret;
    ret.name = jnt.getName();

    H_new_old_successor = getH_new_old(jnt,frameToTip);

    ret.parent_to_joint_origin_transform = toUrdf(H_new_old_predecessor*frameToTip*(H_new_old_successor.Inverse()));

    switch(jnt.getType())
    {
        case KDL::Joint::RotAxis:
        case KDL::Joint::RotX:
        case KDL::Joint::RotY:
        case KDL::Joint::RotZ:
            //using continuos if no joint limits are specified
            ret.type = urdf::Joint::CONTINUOUS;
            //in urdf, the joint axis is expressed in the joint/successor frame
            //in kdl, the joint axis is expressed in the predecessor rame
            ret.axis = toUrdf(((frameToTip).M.Inverse((jnt.JointAxis()))));
        break;
        case KDL::Joint::TransAxis:
        case KDL::Joint::TransX:
        case KDL::Joint::TransY:
        case KDL::Joint::TransZ:
            ret.type = urdf::Joint::PRISMATIC;
            //in urdf, the joint axis is expressed in the joint/successor frame
            //in kdl, the joint axis is expressed in the predecessor rame
            ret.axis = toUrdf((frameToTip.M.Inverse(jnt.JointAxis())));
        break;
        default:
            std::cerr << "[WARN] Converting unknown joint type of joint " << jnt.getTypeName() << " into a fixed joint" << std::endl;
        case KDL::Joint::None:
            ret.type = urdf::Joint::FIXED;
    }
    return ret;
}

// construct inertia
urdf::Inertial toUrdf(KDL::RigidBodyInertia i)
{
  urdf::Inertial ret;
  ret.mass = i.getMass();
  ret.origin = toUrdf(KDL::Frame(KDL::Rotation::Identity(),i.getCOG()));
  // kdl specifies the inertia in the reference frame of the link, the urdf specifies the inertia in the inertia reference frame
  // however the kdl RigidBodyInertia constructor take the inertia with the COG as reference point,
  // but the getInertia
  KDL::RotationalInertia Ic;
  Ic = i.RefPoint(i.getCOG()).getRotationalInertia();
  ret.ixx = Ic.data[0];
  ret.ixy = Ic.data[1];
  ret.ixz = Ic.data[2];
  ret.iyy = Ic.data[4];
  ret.iyz = Ic.data[5];
  ret.izz = Ic.data[8];
  return ret;
}


bool treeToUrdfFile(const string& file, const KDL::Tree& tree, const std::string & robot_name)
{
  TiXmlDocument * urdf_xml;
  if( !treeToUrdfXml(urdf_xml, tree, robot_name) ) return false;
    return urdf_xml->SaveFile(file);
}

/*
bool treeFromParam(const string& param, Tree& tree)
{
  urdf::Model robot_model;
  if (!robot_model.initParam(param)){
    ROS_ERROR("Could not generate robot model");
    return false;
  }
  return treeFromUrdfModel(robot_model, tree);
}

bool treeFromString(const string& xml, Tree& tree)
{
  TiXmlDocument urdf_xml;
  urdf_xml.Parse(xml.c_str());
  return treeFromXml(&urdf_xml, tree);
}
*/

bool treeToUrdfXml(TiXmlDocument* & xml_doc, const KDL::Tree& tree, const std::string & robot_name)
{
    urdf::ModelInterface robot_model;
    if( !treeToUrdfModel(tree,robot_name,robot_model) ) return false;
    xml_doc =  urdf::exportURDF(robot_model);
    return true;
}

bool treeToUrdfModel(const KDL::Tree& tree, const std::string & robot_name, urdf::ModelInterface& robot_model)
{
    robot_model.clear();
    robot_model.name_ = robot_name;

    //Add all links
    KDL::SegmentMap::iterator seg;
    KDL::SegmentMap segs;
    KDL::SegmentMap::const_iterator root_seg;
    root_seg = tree.getRootSegment();
    segs = tree.getSegments();
    for( seg = segs.begin(); seg != segs.end(); seg++ ) {
        if (robot_model.getLink(seg->first))
        {
            logError("link '%s' is not unique.",  seg->first.c_str());
            robot_model.clear();
            return false;
        }
        else
        {
            boost::shared_ptr<urdf::Link> link;
            link.reset(new urdf::Link);

            //add name
            link->name = seg->first;

            //insert link
            robot_model.links_.insert(make_pair(seg->first,link));
            logDebug("successfully added a new link '%s'", link->name.c_str());
        }

        //inserting joint
        //The fake root segment has no joint to add
        if( seg->first != root_seg->first ) {
            KDL::Joint jnt;
            jnt = GetTreeElementSegment(seg->second).getJoint();
            if (robot_model.getJoint(jnt.getName()))
            {
                logError("joint '%s' is not unique.", jnt.getName().c_str());
                robot_model.clear();
                return false;
            }
            else
            {
                boost::shared_ptr<urdf::Joint> joint;
                boost::shared_ptr<urdf::Link> link = robot_model.links_[seg->first];
                //This variable will be set by toUrdf
                KDL::Frame H_new_old_successor;
                KDL::Frame H_new_old_predecessor = getH_new_old(GetTreeElementSegment(GetTreeElementParent(seg->second)->second));
                joint.reset(new urdf::Joint());

                //convert joint
                *joint = toUrdf(jnt, GetTreeElementSegment(seg->second).getFrameToTip(),H_new_old_predecessor,H_new_old_successor);

                //insert parent
                joint->parent_link_name = GetTreeElementParent(seg->second)->first;

                //insert child
                joint->child_link_name = seg->first;

                //insert joint
                robot_model.joints_.insert(make_pair(seg->first,joint));
                logDebug("successfully added a new joint '%s'", jnt.getName().c_str());

                //add inertial, taking in account an eventual change in the link frame
                link->inertial.reset(new urdf::Inertial());
                *(link->inertial) = toUrdf(H_new_old_successor * GetTreeElementSegment(seg->second).getInertia());
            }
        }

    }

    // every link has children links and joints, but no parents, so we create a
    // local convenience data structure for keeping child->parent relations
    std::map<std::string, std::string> parent_link_tree;
    parent_link_tree.clear();

    // building tree: name mapping
    //try
    //{
        robot_model.initTree(parent_link_tree);
    //}
    /*
    catch(ParseError &e)
    {
        logError("Failed to build tree: %s", e.what());
        robot_model.clear();
        return false;
    }*/

    // find the root link
    //try
    //{
        robot_model.initRoot(parent_link_tree);
    //}
    /*
    catch(ParseError &e)
    {
        logError("Failed to find root link: %s", e.what());
        robot_model.reset();
        return false;
    }
    */
    return true;
}

//update topology and parameters
//use only on KDL models obtained from KDL_import from the corresponding urdf tree

bool treeUpdateUrdfModel(const KDL::Tree& tree, urdf::ModelInterface& robot_model)
{
    /*
    KDL::SegmentMap::iterator seg;
    KDL::SegmentMap segs;
    tree.getSegments(segs);

    //Update all links
    for( seg = segs.begin(); seg != segs.end(); seg++ ) {
        if (robot_model.getLink(seg->first))
        {
            boost::shared_ptr<urdf::Link> link = robot_model.links_[seg->first];
            //update inertial
            link->inertial.reset(new urdf::Inertial());
            *(link->inertial) = toUrdf(seg->second.segment.getInertia());

            logDebug("successfully updated link '%s'", link->name.c_str());
        }
        else
        {
            logError("link '%s' not found.",  seg->first.c_str());
            return false;
        }
    }

    for( seg = segs.begin(); seg != segs.end(); seg++ ) {
        KDL::Joint jnt;

        urdf::Joint new_joint;
        jnt = seg->second.segment.getJoint();
        if (robot_model.getJoint(jnt.getName()))
        {
            boost::shared_ptr<urdf::Joint> joint;
            joint = robot_model.joints_[jnt.getName()];

            //convert joint
            new_joint = toUrdf(jnt);

            //update existing joint
            joint->type = new_joint.type;
            joint->axis = new_joint.axis;
            joint->parent_to_joint_origin_transform = new_joint.parent_to_joint_origin_transform;

            logDebug("successfully updated joint '%s'", jnt.getName().c_str());
        }
        else
        {
            logError("joint '%s' not found.",  jnt.getName().c_str());
            return false;
        }
    }
    */
    return false;
}

}

