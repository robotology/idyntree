/*********************************************************************
* Software License Agreement (BSD License)
* 
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

/* Author: Wim Meeussen */

#include "kdl_parser/kdl_parser.hpp"
#include <urdf/model.h>
#include <kdl/frames_io.hpp>
#include <ros/console.h>

using namespace std;
using namespace KDL;

namespace kdl_parser{


// construct vector
Vector toKdl(urdf::Vector3 v)
{
  return Vector(v.x, v.y, v.z);
}

// construct rotation
Rotation toKdl(urdf::Rotation r)
{
  return Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

// construct pose
Frame toKdl(urdf::Pose p)
{
  return Frame(toKdl(p.rotation), toKdl(p.position));
}

// construct joint
Joint toKdl(boost::shared_ptr<urdf::Joint> jnt)
{
  Frame F_parent_jnt = toKdl(jnt->parent_to_joint_origin_transform);

  switch (jnt->type){
  case urdf::Joint::FIXED:{
    return Joint(jnt->name, Joint::None);
  }
  case urdf::Joint::REVOLUTE:{
    Vector axis = toKdl(jnt->axis);
    return Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, Joint::RotAxis);
  }
  case urdf::Joint::CONTINUOUS:{
    Vector axis = toKdl(jnt->axis);
    return Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, Joint::RotAxis);
  }
  case urdf::Joint::PRISMATIC:{
    Vector axis = toKdl(jnt->axis);
    return Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, Joint::TransAxis);
  }
  default:{
    ROS_WARN("Converting unknown joint type of joint '%s' into a fixed joint", jnt->name.c_str());
    return Joint(jnt->name, Joint::None);
  }
  }
  return Joint();
}

// construct inertia
RigidBodyInertia toKdl(boost::shared_ptr<urdf::Inertial> i)
{
  Frame origin = toKdl(i->origin);
  // kdl specifies the inertia in the reference frame of the link, the urdf specifies the inertia in the inertia reference frame
  return origin.M * RigidBodyInertia(i->mass, origin.p, RotationalInertia(i->ixx, i->iyy, i->izz, i->ixy, i->ixz, i->iyz));
}


// recursive function to walk through tree
bool addChildrenToTree(boost::shared_ptr<const urdf::Link> root, Tree& tree)
{
  std::vector<boost::shared_ptr<urdf::Link> > children = root->child_links;
  ROS_DEBUG("Link %s had %i children", root->name.c_str(), (int)children.size());

  // constructs the optional inertia
  RigidBodyInertia inert(0);
  if (root->inertial) 
    inert = toKdl(root->inertial);

  // constructs the kdl joint
  Joint jnt = toKdl(root->parent_joint);

  // construct the kdl segment
  Segment sgm(root->name, jnt, toKdl(root->parent_joint->parent_to_joint_origin_transform), inert);

  // add segment to tree
  tree.addSegment(sgm, root->parent_joint->parent_link_name);

  // recurslively add all children
  for (size_t i=0; i<children.size(); i++){
    if (!addChildrenToTree(children[i], tree))
      return false;
  }
  return true;
}


bool treeFromFile(const string& file, Tree& tree)
{
  TiXmlDocument urdf_xml;
  urdf_xml.LoadFile(file);
  return treeFromXml(&urdf_xml, tree);
}

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

bool treeFromXml(TiXmlDocument *xml_doc, Tree& tree)
{
  urdf::Model robot_model;
  if (!robot_model.initXml(xml_doc)){
    ROS_ERROR("Could not generate robot model");
    return false;
  }
  return treeFromUrdfModel(robot_model, tree);
}


bool treeFromUrdfModel(const urdf::ModelInterface& robot_model, Tree& tree)
{
  tree = Tree(robot_model.getRoot()->name);

  // warn if root link has inertia. KDL does not support this
  if (robot_model.getRoot()->inertial)
    ROS_WARN("The root link %s has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.", robot_model.getRoot()->name.c_str());

  //  add all children
  for (size_t i=0; i<robot_model.getRoot()->child_links.size(); i++)
    if (!addChildrenToTree(robot_model.getRoot()->child_links[i], tree))
      return false;

  return true;
}

}

