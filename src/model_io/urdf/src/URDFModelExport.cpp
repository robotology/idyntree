/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "URDFModelExport.h"

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/SpatialInertia.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/FixedJoint.h>
#include <iDynTree/Model/PrismaticJoint.h>
#include <iDynTree/Model/RevoluteJoint.h>

#include "URDFParsingUtils.h"


#include <iomanip>
#include <fstream>
#include <string>

#include <libxml/tree.h>
#include <libxml/parser.h>

namespace iDynTree
{

/*
 * This file uses extensivly the libxml2's tree API, see
 * http://www.xmlsoft.org/examples/index.html#Tree for more info.
 * Usage of the C-based libxml2 API in C++ means that is necessary to
 * use the BAD_CAST macro for handling const-correctness.
 * See https://stackoverflow.com/questions/45058878/why-libxml2-uses-bad-cast-everywhere-in-a-c-c-code
 * and http://www.xmlsoft.org/html/libxml-xmlstring.html#BAD_CAST for details on this.
 */

/*
 * Add a <origin> URDF element to the specified parent element with the specified transform.
 *
 * If parent_element contains a pointer to a tag <parent_element></parent_element>, this function
 * modifies it to be something like:
 * <parent_element>
 *   <origin xyz="1.0 2.0 3.0" rpy="0.0 1.0 2.0" />
 * </parent_element>
 * where the actual values of the xyz and rpy attributes depend on the input trans argument.
 */
bool exportTransform(const Transform &trans, xmlNodePtr parent_element)
{
    bool ok = true;
    xmlNodePtr origin = xmlNewChild(parent_element, NULL, BAD_CAST "origin", NULL);
    std::string pose_xyz_str;
    ok = ok && vectorToString(trans.getPosition(), pose_xyz_str);
    std::string pose_rpy_str;
    Vector3 pose_rpy = trans.getRotation().asRPY();
    ok = ok && vectorToString(pose_rpy, pose_rpy_str);
    xmlNewProp(origin, BAD_CAST "xyz", BAD_CAST pose_xyz_str.c_str());
    xmlNewProp(origin, BAD_CAST "rpy", BAD_CAST pose_rpy_str.c_str());
    return ok;
}

/*
 * Add a <inertial> URDF element to the specified parent element with the specified 6D inertia.
 *
 * If parent_element contains a pointer to a tag <parent_element></parent_element>, this function modifies it to be something like:
 * <parent_element>
 *   <inertial>
 *     <origin xyz="0 0 0.5" rpy="0 0 0"/>
 *     <mass value="1"/>
 *     <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
 *   </inertial>
 * </parent_element>
 * where the actual values of the origin, mass and inertia element's attributes depend on the input inertia argument.
 */
bool exportInertial(const SpatialInertia &inertia, xmlNodePtr parent_element)
{
    bool ok=true;
    std::string bufStr;
    xmlNodePtr inertial = xmlNewChild(parent_element, NULL, BAD_CAST "inertial", NULL);


    xmlNodePtr mass_xml = xmlNewChild(inertial, NULL, BAD_CAST "mass", NULL);
    ok = ok && doubleToStringWithClassicLocale(inertia.getMass(), bufStr);
    xmlNewProp(mass_xml, BAD_CAST "value", BAD_CAST bufStr.c_str());

    ok = ok && exportTransform(Transform(Rotation::Identity(), inertia.getCenterOfMass()), inertial);

    xmlNodePtr inertia_xml = xmlNewChild(inertial, NULL, BAD_CAST "inertia", NULL);
    RotationalInertiaRaw rotInertia = inertia.getRotationalInertiaWrtCenterOfMass();
    ok = ok && doubleToStringWithClassicLocale(rotInertia(0, 0), bufStr);
    xmlNewProp(inertia_xml, BAD_CAST "ixx", BAD_CAST bufStr.c_str());
    ok = ok && doubleToStringWithClassicLocale(rotInertia(0, 1), bufStr);
    xmlNewProp(inertia_xml, BAD_CAST "ixy", BAD_CAST bufStr.c_str());
    ok = ok && doubleToStringWithClassicLocale(rotInertia(0, 2), bufStr);
    xmlNewProp(inertia_xml, BAD_CAST "ixz", BAD_CAST bufStr.c_str());
    ok = ok && doubleToStringWithClassicLocale(rotInertia(1, 1), bufStr);
    xmlNewProp(inertia_xml, BAD_CAST "iyy", BAD_CAST bufStr.c_str());
    ok = ok && doubleToStringWithClassicLocale(rotInertia(1, 2), bufStr);
    xmlNewProp(inertia_xml, BAD_CAST "iyz", BAD_CAST bufStr.c_str());
    ok = ok && doubleToStringWithClassicLocale(rotInertia(2, 2), bufStr);
    xmlNewProp(inertia_xml, BAD_CAST "izz", BAD_CAST bufStr.c_str());

    return ok;
}

/*
 * Add a <link> URDF element to the specified parent element with the specified link info.
 *
 * Currently this function does not export visual and collision elements.
 *
 * If parent_element contains a pointer to a tag <parent_element></parent_element>, this function modifies it to be something like:
 *
 * <parent_element>
 *   <link name="my_link">
 *     <inertial>
 *       ...
 *     </inertial>
 *
 *  </link>
 * <parent_element/>
 *
 * where the actual values of the added element and attributes depend on the input link and linkName arguments.
 */
bool exportLink(const Link &link, const std::string linkName, xmlNodePtr parent_element)
{
    bool ok = true;
    xmlNodePtr link_xml = xmlNewChild(parent_element, NULL, BAD_CAST "link", NULL);
    xmlNewProp(link_xml, BAD_CAST "name", BAD_CAST linkName.c_str());

    ok = ok && exportInertial(link.getInertia(), link_xml);

    /* TODO(traversaro) : support solid shapes
    for (std::size_t i = 0 ; i < link.visual_array.size() ; ++i)
        exportVisual(*link.visual_array[i], link_xml);
    for (std::size_t i = 0 ; i < link.collision_array.size() ; ++i)
        exportCollision(*link.collision_array[i], link_xml);
    */

    return ok;
}


/**
 * Add a <joint> URDF element to the specified parent element with the specified joint info.
 *
 * At the moment, only the URDF joint of types revolute, continuous, prismatic and fixed are supported, as they are the only one supported
 * in the iDynTree::Model class. Furthermore, it is not supported the export of joint limits and friction.
 *
 * If parent_element contains a pointer to a tag <parent_element></parent_element>, this function modifies it to be something like:
 *
 * <parent_element>
 *   <joint name="my_joint" type="floating">
 *      <origin xyz="0 0 1" rpy="0 0 3.1416"/>
 *      <parent link="link1"/>
 *      <child link="link2"/>
 *      <calibration rising="0.0"/>
 *      <dynamics damping="0.0" friction="0.0"/>
 *      <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
 *   </joint>
 * <parent_element/>
 *
 * where the actual values of the added element and attributes depend on the input joint, parentLink and childLink arguments.
 */
bool exportJoint(IJointConstPtr joint, LinkConstPtr parentLink, LinkConstPtr childLink, const Model& model, xmlNodePtr parent_element)
{
    bool ok=true;
    xmlNodePtr joint_xml = xmlNewChild(parent_element, NULL, BAD_CAST "joint", NULL);
    xmlNewProp(joint_xml, BAD_CAST "name", BAD_CAST model.getJointName(joint->getIndex()).c_str());

    if (dynamic_cast<const FixedJoint*>(joint))
    {
        xmlNewProp(joint_xml, BAD_CAST "type", BAD_CAST "fixed");
    }
    else if (dynamic_cast<const RevoluteJoint*>(joint))
    {
        if(joint->hasPosLimits())
        {
            xmlNewProp(joint_xml, BAD_CAST "type", BAD_CAST "revolute");
        }
        else
        {
            xmlNewProp(joint_xml, BAD_CAST "type", BAD_CAST "continuous");
        }
    }
    else if (dynamic_cast<const PrismaticJoint*>(joint))
    {
        xmlNewProp(joint_xml, BAD_CAST "type", BAD_CAST "prismatic");
    }
    else
    {
        std::cerr << "[ERROR] URDFModelExport: Impossible to convert joint of type "
                  <<  typeid(joint).name() << " to a URDF joint " << std::endl;
        return false;
    }

    // origin
    exportTransform(joint->getRestTransform(parentLink->getIndex(), childLink->getIndex()), joint_xml);

    if (joint->getNrOfDOFs() != 0)
    {
        Axis axis;

        // Check that the axis of the joint is supported by URDF
        if (dynamic_cast<const RevoluteJoint*>(joint))
        {
            const RevoluteJoint* revJoint = dynamic_cast<const RevoluteJoint*>(joint);
            axis = revJoint->getAxis(childLink->getIndex());
        }
        else if (dynamic_cast<const PrismaticJoint*>(joint))
        {
            const PrismaticJoint* prismJoint = dynamic_cast<const PrismaticJoint*>(joint);
            axis = prismJoint->getAxis(childLink->getIndex());
        }
        else
        {
            std::cerr << "[ERROR] URDFModelExport: Impossible to convert joint of type "
                      <<  typeid(joint).name() << " to a URDF joint " << std::endl;
            return false;
        }

        // Check that the axis is URDF-compatible
        if (toEigen(axis.getOrigin()).norm() > 1e-7)
        {
            std::cerr << "[ERROR] URDFModelExport: Impossible to convert joint "
                      <<   model.getJointName(joint->getIndex()) << " to a URDF joint without moving the link frame of link "
                      << model.getLinkName(childLink->getIndex()) << " , because its origin is "
                      << axis.getOrigin().toString()  << std::endl;
            return false;
        }

        // axis
        xmlNodePtr axis_xml = xmlNewChild(joint_xml, NULL, BAD_CAST "axis", NULL);
        std::string bufStr;
        ok = ok && vectorToString(axis.getDirection(), bufStr);
        xmlNewProp(axis_xml, BAD_CAST "xyz", BAD_CAST bufStr.c_str());
    }

    // parent
    xmlNodePtr parent_xml = xmlNewChild(joint_xml, NULL, BAD_CAST "parent", NULL);
    xmlNewProp(parent_xml, BAD_CAST "link", BAD_CAST model.getLinkName(parentLink->getIndex()).c_str());

    // child
    xmlNodePtr child_xml = xmlNewChild(joint_xml, NULL, BAD_CAST "child", NULL);
    xmlNewProp(child_xml, BAD_CAST "link", BAD_CAST model.getLinkName(childLink->getIndex()).c_str());

    // TODO(traversaro) : handle joint limits and friction

    return ok;
}

/*
 * Add a <link> and <joint> URDF elements to the specified parent element for the specified additional frame.
 *
 * URDF does not have the concept of frame (see https://discourse.ros.org/t/urdf-ng-link-and-frame-concepts/56),
 * so for each additional frame we need to add a URDF "fake" mass-less link.
 *
 * If parent_element contains a pointer to a tag <parent_element></parent_element>, this function modifies it to be something like:
 *
 * <parent_element>
 *   <link name="<frame_name>"/>
 *   <joint name="<frame_name>_fixed_joint" type="fixed">
 *      <origin xyz="0.1 0.2 0.3" rpy="0 0 3.1416"/>
 *      <parent link="<link_name>"/>
 *      <child link="<frame_name>"/>
 *   </joint>
 * <parent_element/>
 *
 * where the actual values of the added element and attributes depend on the input frame_name, link_H_frame, parent_link_name and direction_options arguments.
 */
enum exportAdditionalFrameDirectionOption {
    FAKE_LINK_IS_CHILD,
    FAKE_LINK_IS_PARENT
};
bool exportAdditionalFrame(const std::string frame_name, Transform link_H_frame, const std::string link_name, exportAdditionalFrameDirectionOption direction_option, xmlNodePtr parent_element)
{
    bool ok=true;

    // Export fake link
    xmlNodePtr link_xml = xmlNewChild(parent_element, NULL, BAD_CAST "link", NULL);
    xmlNewProp(link_xml, BAD_CAST "name", BAD_CAST frame_name.c_str());

    // Export fake joint
    xmlNodePtr joint_xml = xmlNewChild(parent_element, NULL, BAD_CAST "joint", NULL);
    std::string fake_joint_name = frame_name + "_fixed_joint";
    xmlNewProp(joint_xml, BAD_CAST "name", BAD_CAST fake_joint_name.c_str());
    xmlNewProp(joint_xml, BAD_CAST "type", BAD_CAST "fixed");

    // origin
    exportTransform(link_H_frame, joint_xml);

    std::string parent_of_fake_joint, child_of_fake_joint;

    if (direction_option == FAKE_LINK_IS_CHILD) {
        parent_of_fake_joint = link_name;
        child_of_fake_joint = frame_name;
    } else {
        // direction_option == FAKE_LINK_IS_PARENT
        parent_of_fake_joint = frame_name;
        child_of_fake_joint = link_name;
    }

    // parent
    xmlNodePtr parent_xml = xmlNewChild(joint_xml, NULL, BAD_CAST "parent", NULL);
    xmlNewProp(parent_xml, BAD_CAST "link", BAD_CAST parent_of_fake_joint.c_str());

    // child
    xmlNodePtr child_xml = xmlNewChild(joint_xml, NULL, BAD_CAST "child", NULL);
    xmlNewProp(child_xml, BAD_CAST "link", BAD_CAST child_of_fake_joint.c_str());

    return ok;
}

bool URDFStringFromModel(const iDynTree::Model & model,
                         std::string & urdf_string,
                         const ModelExporterOptions options)
{
    bool ok = true;

    xmlDocPtr urdf_xml = xmlNewDoc(BAD_CAST "1.0");
    xmlNodePtr robot = xmlNewNode(NULL, BAD_CAST "robot");
    // TODO(traversaro) properly export this :)
    xmlNewProp(robot, BAD_CAST "name", BAD_CAST "iDynTreeURDFModelExportModelName");
    xmlDocSetRootElement(urdf_xml, robot);

    // TODO(traversaro) : We are assuming that the model has no loops,
    //                    we should add a check for this

    // URDF assumes a directed tree of the multibody model structure, so we need to assume that a given link is
    // the base
    std::string baseLink = options.baseLink;
    if (baseLink.empty())
    {
        baseLink = model.getLinkName(model.getDefaultBaseLink());
    }

    LinkIndex baseLinkIndex = model.getLinkIndex(baseLink);
    if (baseLinkIndex == LINK_INVALID_INDEX)
    {
        std::cerr << "[ERROR] URDFStringFromModel : specified baseLink " << baseLink << " is not part of the model" << std::endl;
        xmlFreeDoc(urdf_xml);
        return false;
    }

    // If the base link has at least an additional frame, add it as parent URDF link
    // as a workaround for https://github.com/ros/kdl_parser/issues/27
    FrameIndex baseFakeLinkFrameIndex = FRAME_INVALID_INDEX;
    std::vector<FrameIndex> frameIndices;
    ok = model.getLinkAdditionalFrames(baseLinkIndex, frameIndices);
    if (ok && frameIndices.size() >= 1) {
        baseFakeLinkFrameIndex = frameIndices[0];
        ok = ok && exportAdditionalFrame(model.getFrameName(baseFakeLinkFrameIndex),
                                         model.getFrameTransform(baseFakeLinkFrameIndex),
                                         model.getLinkName(model.getFrameLink(baseFakeLinkFrameIndex)),
                                         FAKE_LINK_IS_PARENT,
                                         robot);
    }


    // Create a Traversal
    Traversal exportTraversal;
    if (!model.computeFullTreeTraversal(exportTraversal, baseLinkIndex))
    {
        std::cerr << "[ERROR] URDFStringFromModel : error in computeFullTreeTraversal" << std::endl;
        xmlFreeDoc(urdf_xml);
        return false;
    }


    // Export links and joints following the traversal
    for (TraversalIndex trvIdx=0; trvIdx < static_cast<TraversalIndex>(exportTraversal.getNrOfVisitedLinks()); trvIdx++)
    {
        LinkConstPtr visitedLink = exportTraversal.getLink(trvIdx);
        std::string visitedLinkName = model.getLinkName(visitedLink->getIndex());

        // Export parent joint, if this is not the base
        if (trvIdx != 0)
        {
            LinkConstPtr parentLink = exportTraversal.getParentLink(trvIdx);
            IJointConstPtr parentJoint = exportTraversal.getParentJoint(trvIdx);
            ok = ok && exportJoint(parentJoint, parentLink, visitedLink,  model, robot);
        }

        // Export link
        ok = ok && exportLink(*visitedLink, visitedLinkName, robot);
    }

    // Export all the additional frames that are not parent of the base link
    for (FrameIndex frameIndex=model.getNrOfLinks(); frameIndex < static_cast<FrameIndex>(model.getNrOfFrames()); frameIndex++)
    {
        if (frameIndex != baseFakeLinkFrameIndex) {
            ok = ok && exportAdditionalFrame(model.getFrameName(frameIndex),
                                             model.getFrameTransform(frameIndex),
                                             model.getLinkName(model.getFrameLink(frameIndex)),
                                             FAKE_LINK_IS_CHILD,
                                             robot);
        }
    }


    xmlChar *xmlbuff=0;
    int buffersize=0;
    xmlDocDumpFormatMemory(urdf_xml, &xmlbuff, &buffersize, 1);
    urdf_string.resize(buffersize);
    std::memcpy((void*)urdf_string.data(), (void*)xmlbuff, buffersize);

    xmlFree(xmlbuff);
    xmlFreeDoc(urdf_xml);

    return ok;
}


bool URDFFromModel(const iDynTree::Model & model,
                   const std::string & urdf_filename,
                   const ModelExporterOptions options)
{
    std::ofstream ofs(urdf_filename.c_str());

    if( !ofs.is_open() )
    {
        std::cerr << "[ERROR] iDynTree::URDFFromModel : error opening file "
                  << urdf_filename << std::endl;
        return false;
    }

    std::string xml_string;
    if (!URDFStringFromModel(model, xml_string, options))
    {
        return false;
    }

    // Write string to file
    ofs << xml_string;
    ofs.close();

    return true;
}


}
