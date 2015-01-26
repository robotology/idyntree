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
/* Modified by: Silvio Traversaro */

#ifndef KDL_IMPORT_H
#define KDL_IMPORT_H

#include <string>
#include <vector>

class TiXmlDocument;
namespace urdf {
    class ModelInterface;
}

namespace KDL {
    class Tree;
    class JntArray;
}

namespace kdl_format_io{

/** Constructs a KDL tree from a file, given the file name
 * \param file The filename from where to read the xml
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
bool treeFromUrdfFile(const std::string& file, KDL::Tree& tree, const bool consider_root_link_inertia=false);

/** Constructs a KDL tree from the parameter server, given the parameter name
 * \param param the name of the parameter on the parameter server
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
//bool treeFromParam(const std::string& param, KDL::Tree& tree);

/** Constructs a KDL tree from a string containing xml
 * \param xml A string containting the xml description of the robot
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
bool treeFromUrdfString(const std::string& xml, KDL::Tree& tree, const bool consider_root_link_inertia=false);


/** Constructs a KDL tree from a TiXmlDocument
 * \param xml_doc The TiXmlDocument containting the xml description of the robot
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
//bool treeFromXml(TiXmlDocument *xml_doc, KDL::Tree& tree);

/** Constructs a KDL tree from a URDF robot model
 * \param robot_model The URDF robot model
 * \param tree The resulting KDL Tree
 * \param consider_root_link_inertia optional (default false) if true parse the first link 
 *                  of the robot model as a real link, and introduces a dummy link connected to it
 *                 with a fixed joint to overcome the the fact that KDL does not support inertia in the first link
 * returns true on success, false on failure
 */
bool treeFromUrdfModel(const urdf::ModelInterface& robot_model, KDL::Tree& tree, const bool consider_root_link_inertia=false);

/**
 * \todo TODO FIXME write proper JointLimit/JointPosLimits to replace use of joint_names,min,max 
 *
 */
bool jointPosLimitsFromUrdfFile(const std::string& file, std::vector<std::string> & joint_names, KDL::JntArray & min, KDL::JntArray & max);
bool jointPosLimitsFromUrdfString(const std::string& urdf_xml,std::vector<std::string> & joint_names, KDL::JntArray & min, KDL::JntArray & max);
bool jointPosLimitsFromUrdfModel(const urdf::ModelInterface& robot_model, std::vector<std::string> & joint_names, KDL::JntArray & min, KDL::JntArray & max);

}

#endif
