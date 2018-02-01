/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

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
