/*********************************************************************
* Software License Agreement (BSD License)
*  
*  Copyright (c) 2013, Istituto Italiano di Tecnologia,
*   CoDyCo project
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

#ifndef KDL_EXPORT_H
#define KDL_EXPORT_H

#include <string>

class TiXmlDocument;

namespace KDL {
    class Tree;
}

namespace urdf {
    class ModelInterface;
}

namespace iDynTree{

/** Constructs a URDF file, given a KDL::Tree
 * \param file The filename from where to read the xml
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
bool treeToUrdfFile(const std::string& file, const KDL::Tree& tree, const std::string & robot_name="URDF_generated_by_kdl_format_io");

/** Constructs a KDL tree from the parameter server, given the parameter name
 * \param param the name of the parameter on the parameter server
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
//bool treeToParam(const std::string& param, KDL::Tree& tree);

/** Constructs a KDL tree from a string containing xml
 * \param xml A string containting the xml description of the robot
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
//bool treeToString(const std::string& xml, KDL::Tree& tree);

/** Constructs a URDF TiXmlDocument given a KDL::Tree
 * \param xml_doc The TiXmlDocument containting the xml description of the robot
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
bool treeToUrdfXml(TiXmlDocument * & xml_doc,  const KDL::Tree& tree, const std::string & robot_name="URDF_generated_by_kdl_format_io");

/** Update a URDF robot model with geometric and inertial parameters from a KDL tree 
 * \param tree The KDL Tree
 * \param robot_model input, output parameter, the resulting URDF robot model
 * returns true on success, false on failure
 */
bool treeUpdateUrdfModel(const KDL::Tree& tree, urdf::ModelInterface& robot_model);


/** Constructs a URDF robot model from a KDL tree
 *  \note the URDF specs impose some costraints on the location of the link
 *  frames (it is required that the origin of the frame is on the axis of the joint).
 *  If the KDL::Tree frame does not respect these constraints, they are moved.
 * 
 * \param tree The KDL Tree
 * \param robot_name the name of the KDL Tree
 * \param robot_model The resulting URDF robot model
 * returns true on success, false on failure
 */
bool treeToUrdfModel(const KDL::Tree& tree, const std::string & robot_name, urdf::ModelInterface& robot_model);

}

#endif
