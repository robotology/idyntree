/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
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

namespace iDynTree{

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

/**
 * The URDF format does not support the concept of frames. For this reason
 * people usually attach with fixed joints "fake" leaf links with a zero
 * mass to their models, to represent frames attached to a given link.
 *
 * This function encodes an heuristics for extracting this kind of information:
 * it returns as frame "fake links" all the URDF leaf links that have zero mass
 * and are attach to their parents through a fixed joint.
 *
 * \warning This is just a workaround for a problem in URDF expressiveness,
 *           but it can fail if the model is not following the assumptions of
 *           the function.
 *
 * \param[in] tree the KDL::Tree
 * \param[out] framesNames the names of the link that are actually "fake links" used to represent frames
 * \param[out] parentLinkNames for each frame, the name of the parent link to which the frame is attached
 *
 */
bool framesFromKDLTree(const KDL::Tree& tree,
                       std::vector<std::string>& framesNames,
                       std::vector<std::string>& parentLinkNames);


}

#endif
