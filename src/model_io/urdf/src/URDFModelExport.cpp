/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <iDynTree/ModelIO/URDFModelExport.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/SpatialInertia.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/FixedJoint.h>
#include <iDynTree/Model/PrismaticJoint.h>
#include <iDynTree/Model/RevoluteJoint.h>

#include "URDFParsingUtils.h"

#include <tinyxml.h>

#include <iomanip>
#include <fstream>

namespace iDynTree
{

bool exportTransform(const Transform &trans, TiXmlElement* xml)
{
    bool ok = true;
    TiXmlElement *origin = new TiXmlElement("origin");
    std::string pose_xyz_str;
    ok = ok && vectorToString(trans.getPosition(), pose_xyz_str);
    std::string pose_rpy_str;
    Vector3 pose_rpy = trans.getRotation().asRPY();
    ok = ok && vectorToString(pose_rpy, pose_rpy_str);
    origin->SetAttribute("xyz", pose_xyz_str);
    origin->SetAttribute("rpy", pose_rpy_str);
    xml->LinkEndChild(origin);
    return ok;
}

bool exportInertial(const SpatialInertia &i, TiXmlElement *xml)
{
    bool ok=true;
    std::string bufStr;
    TiXmlElement *inertial_xml = new TiXmlElement("inertial");

    TiXmlElement *mass_xml = new TiXmlElement("mass");
    ok = ok && doubleToStringWithClassicLocale(i.getMass(), bufStr);
    mass_xml->SetAttribute("value", bufStr);
    inertial_xml->LinkEndChild(mass_xml);

    ok = ok && exportTransform(Transform(Rotation::Identity(), i.getCenterOfMass()), inertial_xml);

    TiXmlElement *inertia_xml = new TiXmlElement("inertia");
    RotationalInertiaRaw rotInertia = i.getRotationalInertiaWrtCenterOfMass();
    ok = ok && doubleToStringWithClassicLocale(rotInertia(0, 0), bufStr);
    inertia_xml->SetAttribute("ixx", bufStr);
    ok = ok && doubleToStringWithClassicLocale(rotInertia(0, 1), bufStr);
    inertia_xml->SetAttribute("ixy", bufStr);
    ok = ok && doubleToStringWithClassicLocale(rotInertia(0, 2), bufStr);
    inertia_xml->SetAttribute("ixz", bufStr);
    ok = ok && doubleToStringWithClassicLocale(rotInertia(1, 1), bufStr);
    inertia_xml->SetAttribute("iyy", bufStr);
    ok = ok && doubleToStringWithClassicLocale(rotInertia(1, 2), bufStr);
    inertia_xml->SetAttribute("iyz", bufStr);
    ok = ok && doubleToStringWithClassicLocale(rotInertia(2, 2), bufStr);
    inertia_xml->SetAttribute("izz", bufStr);
    inertial_xml->LinkEndChild(inertia_xml);

    xml->LinkEndChild(inertial_xml);

    return ok;
}

bool exportLink(const Link &link, const std::string linkName, TiXmlElement* xml)
{
    bool ok = true;
    TiXmlElement * link_xml = new TiXmlElement("link");
    link_xml->SetAttribute("name", linkName.c_str());

    ok = ok && exportInertial(link.getInertia(), link_xml);

    /* TODO(traversaro) : support solid shapes
    for (std::size_t i = 0 ; i < link.visual_array.size() ; ++i)
        exportVisual(*link.visual_array[i], link_xml);
    for (std::size_t i = 0 ; i < link.collision_array.size() ; ++i)
        exportCollision(*link.collision_array[i], link_xml);
    */

    xml->LinkEndChild(link_xml);

    return ok;
}

bool exportJoint(IJointConstPtr joint, LinkConstPtr parentLink, LinkConstPtr childLink, const Model& model, TiXmlElement* xml)
{
    bool ok=true;
    TiXmlElement * joint_xml = new TiXmlElement("joint");
    joint_xml->SetAttribute("name", model.getJointName(joint->getIndex()));
    if (dynamic_cast<const FixedJoint*>(joint))
    {
        joint_xml->SetAttribute("type", "fixed");
    }
    else if (dynamic_cast<const RevoluteJoint*>(joint))
    {
        if(joint->hasPosLimits())
        {
            joint_xml->SetAttribute("type", "revolute");
        }
        else
        {
            joint_xml->SetAttribute("type", "continuous");
        }
    }
    else if (dynamic_cast<const PrismaticJoint*>(joint))
    {
        joint_xml->SetAttribute("type", "revolute");
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
        TiXmlElement *axis_xml = new TiXmlElement("axis");
        std::string bufStr;
        ok = ok && vectorToString(axis.getDirection(), bufStr);
        axis_xml->SetAttribute("xyz", bufStr);
        joint_xml->LinkEndChild(axis_xml);
    }

    // parent
    TiXmlElement * parent_xml = new TiXmlElement("parent");
    parent_xml->SetAttribute("link", model.getLinkName(parentLink->getIndex()));
    joint_xml->LinkEndChild(parent_xml);

    // child
    TiXmlElement * child_xml = new TiXmlElement("child");
    child_xml->SetAttribute("link", model.getLinkName(childLink->getIndex()));
    joint_xml->LinkEndChild(child_xml);

    // TODO(traversaro) : handle joint limits and friction

    xml->LinkEndChild(joint_xml);
    return ok;
}

bool URDFStringFromModel(const iDynTree::Model & model,
                         std::string & urdf_string,
                         const URDFExporterOptions options)
{
    bool ok = true;

    TiXmlDocument *urdfXml = new TiXmlDocument();

    TiXmlElement *robot = new TiXmlElement("robot");
    // TODO(traversaro) properly export this
    robot->SetAttribute("name", "iDynTreeURDFModelExportModelName");
    urdfXml->LinkEndChild(robot);

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
        delete urdfXml;
        return false;
    }

    // Create a Traversal
    Traversal exportTraversal;
    if (!model.computeFullTreeTraversal(exportTraversal, baseLinkIndex))
    {
        std::cerr << "[ERROR] URDFStringFromModel : error in computeFullTreeTraversal" << std::endl;
        delete urdfXml;
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

    // TODO(traversaro) : export additional frames
    TiXmlPrinter printer;
    printer.SetIndent("    ");
    urdfXml->Accept( &printer );
    urdf_string = printer.CStr();

    delete urdfXml;

    return ok;
}


bool URDFFromModel(const iDynTree::Model & model,
                   const std::string & urdf_filename,
                   const URDFExporterOptions options)
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
