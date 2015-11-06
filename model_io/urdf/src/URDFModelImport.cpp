/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <iDynTree/ModelIO/URDFModelImport.h>

#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/Transform.h>

#include <iDynTree/Model/FixedJoint.h>
#include <iDynTree/Model/RevoluteJoint.h>
#include <iDynTree/Model/Model.h>

#include <tinyxml.h>
#include <iDynTree/ModelIO/impl/urdf_export.hpp>

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <set>

namespace iDynTree
{

double URDFImportTol = 1e-9;

bool inline stringToDouble(const std::string & inStr, double & outDouble)
{
    outDouble = std::atof(inStr.c_str());
    return true;
}

bool inline stringToInt(const std::string & inStr, int & outInt)
{
    outInt = std::atoi(inStr.c_str());
    return true;
}

bool inline stringToUnsignedInt(const std::string & inStr, unsigned int & outInt)
{
    outInt = (unsigned int)std::atoi(inStr.c_str());
    return true;
}

std::string inline intToString(const int inInt)
{
    std::stringstream ss;
    ss << inInt;
    return ss.str();
}

/**
 * Split string along spaces
 */
bool inline splitString(const std::string & inStr, std::vector<std::string> & pieces)
{
    std::istringstream iss(inStr);

    pieces.resize(0);

    do
    {
        std::string sub;
        iss >> sub;
        if( sub != "" )
        {
            pieces.push_back(sub);
        }
    } while (iss);


    return true;
}

/**
 *
 */
bool vector3FromString(const std::string & vector_str, Vector3 & out)
{
    std::vector<std::string> pieces;
    std::vector<double> xyz;
    splitString(vector_str,pieces);
    for (unsigned int i = 0; i < pieces.size(); ++i)
    {
      if (pieces[i] != ""){
       double newDouble;
       if( stringToDouble(pieces[i],newDouble) )
       {
          xyz.push_back(newDouble);
       }
       else
       {
           std::string errStr = "Unable to parse component [" + pieces[i] + "] to a double (while parsing a vector value)";
           reportError("","vector3FromString",errStr.c_str());
           return false;
       }
      }
    }

    if (xyz.size() != 3)
    {
        std::string errStr = "Parser found " + intToString(xyz.size())  + " elements but 3 expected while parsing vector [" + vector_str + "]";
        reportError("","vector3FromString",errStr.c_str());
        return false;
    }

    out(0) = xyz[0];
    out(1) = xyz[1];
    out(2) = xyz[2];

    return true;
}


bool transformFromURDFXML(TiXmlElement* xml,
                          Transform & transform)
{
    // Default transform is identify
    transform = Transform::Identity();

    if (xml)
    {
        const char* xyz_str = xml->Attribute("xyz");
        if (xyz_str != NULL)
        {
            Position xyz;
            if( vector3FromString(xyz_str,xyz) )
            {
                transform.setPosition(xyz);
            }
            else
            {
                return false;
            }
        }

        const char* rpy_str = xml->Attribute("rpy");
        if (rpy_str != NULL)
        {
            Vector3 rpy;
            if( vector3FromString(rpy_str,rpy) )
            {
                transform.setRotation(Rotation::RPY(rpy(0),rpy(1),rpy(2)));
            }
            else
            {
                return false;
            }
        }
    }
    return true;
}


bool inertiaFromURDFXML(TiXmlElement * inertiaXml,
                        SpatialInertia & inertia)
{
    // Parse link_T_com transform
    // by default is the identity
    Transform link_T_com = Transform::Identity();

    TiXmlElement *o = inertiaXml->FirstChildElement("origin");
    if (o)
    {
        if (!transformFromURDFXML(o, link_T_com))
        return false;
    }

    TiXmlElement *mass_xml = inertiaXml->FirstChildElement("mass");
    if (!mass_xml)
    {
        reportError("","inertiaFromURDFXML","Inertial element must have a mass element");
        return false;
    }
    if (!mass_xml->Attribute("value"))
    {
        reportError("","inertiaFromURDFXML","Inertial: mass element must have value attribute");
        return false;
    }

    double mass;
    if( !stringToDouble(mass_xml->Attribute("value"),mass) )
    {
        std::stringstream stm;
        stm << "Inertial: mass [" << mass_xml->Attribute("value")
            << "] is not a float";
        reportError("","inertiaFromURDFXML",stm.str().c_str());
        return false;
    }

    TiXmlElement *inertia_xml = inertiaXml->FirstChildElement("inertia");
    if (!inertia_xml)
    {
        reportError("","inertiaFromURDFXML","Inertial element must have inertia element");
        return false;
    }
    if (!(inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") && inertia_xml->Attribute("ixz") &&
            inertia_xml->Attribute("iyy") && inertia_xml->Attribute("iyz") &&
            inertia_xml->Attribute("izz")))
    {
        reportError("","inertiaFromURDFXML","Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes");
        return false;
    }

    double ixx, ixy, ixz, iyy, iyz, izz;
    if( !stringToDouble(inertia_xml->Attribute("ixx"),ixx)
        || !stringToDouble(inertia_xml->Attribute("ixy"),ixy)
        || !stringToDouble(inertia_xml->Attribute("ixz"),ixz)
        || !stringToDouble(inertia_xml->Attribute("iyy"),iyy)
        || !stringToDouble(inertia_xml->Attribute("iyz"),iyz)
        || !stringToDouble(inertia_xml->Attribute("izz"),izz) )

    {
        std::stringstream stm;
        stm << "Inertial: one of the inertia elements is not a valid double:"
            << " ixx [" << inertia_xml->Attribute("ixx") << "]"
            << " ixy [" << inertia_xml->Attribute("ixy") << "]"
            << " ixz [" << inertia_xml->Attribute("ixz") << "]"
            << " iyy [" << inertia_xml->Attribute("iyy") << "]"
            << " iyz [" << inertia_xml->Attribute("iyz") << "]"
            << " izz [" << inertia_xml->Attribute("izz") << "]";
        reportError("","inertiaFromURDFXML",stm.str().c_str());
        return false;
    }

    // Create rotational inertia
    double rotInertiaWrtComData[3*3] = { ixx, ixy, ixz,
                                         ixy, iyy, iyz,
                                         ixz, iyz, izz};
    RotationalInertiaRaw rotInertiaWrtCom(rotInertiaWrtComData,3,3);

    // We need to rotate the inertia in the link reference frame
    // and use a special constructor-like function that takes in
    // to account that is expressed with respect to the com
    Position com_wrt_link = link_T_com.getPosition();
    Rotation link_R_com   = link_T_com.getRotation();
    inertia.fromRotationalInertiaWrtCenterOfMass(mass,com_wrt_link,link_R_com*rotInertiaWrtCom);

    return true;
}

bool linkFromURDFXML(TiXmlElement* linkXml,
                 Link & link,
                 std::string & linkName)
{
    const char *name_char = linkXml->Attribute("name");
    if (!name_char)
    {
        reportError("","linkFromURDFXML","No name given for a link in the model.");
        return false;
    }
    linkName = std::string(name_char);

    // Inertial (optional)
    TiXmlElement *i = linkXml->FirstChildElement("inertial");
    SpatialInertia inertia;
    if (i)
    {
        if (!inertiaFromURDFXML(i,inertia))
        {
            std::string errStr = "Could not parse inertial element for link " + linkName;
            reportError("","linkFromURDFXML",errStr.c_str());
            return false;
        }
    }
    link.setInertia(inertia);

    return true;
}

bool axisFromURDFXML(TiXmlElement* axisXml,
                     Axis         & axis)
{
    if (!axisXml || !(axisXml->Attribute("xyz")))
    {
        axis = Axis(Direction(1.0, 0.0, 0.0),Position(0.0,0.0,0.0));
    }
    else
    {
        const char* xyz_str = axisXml->Attribute("xyz");
        Vector3 xyz;
        if( vector3FromString(xyz_str,xyz) )
        {
            axis = Axis(Direction(xyz(0), xyz(1), xyz(2)),Position(0.0,0.0,0.0));
        }
        else
        {
            return false;
        }
    }

    return true;
}

/**
 * This function allocated dinamically the joint.
 * The joint should be then deleted by the parser
 * after it has been added to the model.
 *
 */
bool jointFromURDFXML(const Model & model,
                      TiXmlElement* jointXml,
                      IJointPtr & p_joint,
                      std::string & jointName,
                      std::string & parentLinkName,
                      std::string & childLinkName,
                      std::string & jointType)
{
    // reset p_joint
    p_joint = 0;
    // Get Joint Name
    const char *name = jointXml->Attribute("name");
    if (!name)
    {
        reportError("","jointFromURDFXML","unnamed joint found");
        return false;
    }
    jointName = name;

    // Get transform from Parent Link to Joint Frame
    Transform parent_T_joint;
    TiXmlElement *origin_xml = jointXml->FirstChildElement("origin");
    if (!origin_xml)
    {
        parent_T_joint = Transform::Identity();
    }
    else
    {
        if (!transformFromURDFXML(origin_xml,parent_T_joint))
        {
            std::string errStr = "Malformed parent origin element for joint " + jointName;
            reportError("","jointFromURDFXML",errStr.c_str());
            return false;
        }
    }

    // Get Parent Link
    TiXmlElement *parent_xml = jointXml->FirstChildElement("parent");
    if (parent_xml)
    {
        const char *pname = parent_xml->Attribute("link");
        if (!pname)
        {
            std::string errStr = "No parent specified for joint " + jointName;
            reportError("","jointFromURDFXML",errStr.c_str());
            return false;
        }
        else
        {
            parentLinkName = std::string(pname);
        }
    }

    // Get Child Link
    TiXmlElement *child_xml = jointXml->FirstChildElement("child");
    if (child_xml)
    {
        const char *pname = child_xml->Attribute("link");
        if (!pname)
        {
            std::string errStr = "No child specified for joint " + jointName;
            reportError("","jointFromURDFXML",errStr.c_str());
            return false;
        }
        else
        {
            childLinkName = std::string(pname);
        }
    }

    assert(parentLinkName != childLinkName);

    // Get Joint type
    const char* type_char = jointXml->Attribute("type");
    if (!type_char)
    {
        std::string errStr = "No child specified for joint " + jointName;
        reportError("","jointFromURDFXML",errStr.c_str());
        return false;
    }

    std::string type_str = type_char;
    jointType = type_str;
    if (type_str == "planar" ||
        type_str == "floating" ||
        type_str == "prismatic" )
    {
        std::string errStr = "Joint " + jointName + " has type " + type_str + " that is not currently supported by iDynTree";
        reportError("","jointFromURDFXML",errStr.c_str());
        return false;
    }
    else if (type_str == "fixed" ||
             type_str == "revolute"  ||
             type_str == "continuous")
    {
        // perfect, type supported by iDynTree, parsing happening later
    }
    else
    {
        std::string errStr = "Joint " + jointName + " has unknown type " + type_str;
        reportError("","jointFromURDFXML",errStr.c_str());
        return false;
    }

    // Get indeces in model for the links involved in the joint
    LinkIndex parentLinkIndex = model.getLinkIndex(parentLinkName);

    if( parentLinkIndex == LINK_INVALID_INDEX )
    {
        std::string errStr = "Joint " + jointName + " has unknown parent " + parentLinkName;
        reportError("","jointFromURDFXML",errStr.c_str());
        return false;
    }

    LinkIndex childLinkIndex  = model.getLinkIndex(childLinkName);

    if( childLinkIndex == LINK_INVALID_INDEX )
    {
        std::string errStr = "Joint " + jointName + " has unknown child " + childLinkName;
        reportError("","jointFromURDFXML",errStr.c_str());
        return false;
    }

    assert(parentLinkIndex != childLinkIndex);

     // Get Joint Axis
    Axis axis_wrt_childLink;
    if (type_str != "fixed" && type_str != "floating")
    {
        TiXmlElement *axis_xml = jointXml->FirstChildElement("axis");
        axisFromURDFXML(axis_xml,axis_wrt_childLink);
    }

    // Actually allocate a joint
    if( type_str == "fixed" )
    {
        p_joint = new FixedJoint(parentLinkIndex,childLinkIndex,parent_T_joint);
    }
    else if ( type_str == "revolute"
           || type_str == "continuous" )
    {
        p_joint = new RevoluteJoint(parentLinkIndex,childLinkIndex,
                                    parent_T_joint,parent_T_joint*axis_wrt_childLink);
    }

    assert(p_joint != 0);
    return true;

    // Get limit
    /*
    TiXmlElement *limit_xml = config->FirstChildElement("limit");
    if (limit_xml)
    {
        resetPtr(joint.limits,new JointLimits());
        if (!parseJointLimits(*joint.limits, limit_xml))
        {
        logError("Could not parse limit element for joint [%s]", joint.name.c_str());
        resetPtr(joint.limits);
        return false;
        }
    }
    else if (joint.type == Joint::REVOLUTE)
    {
        logError("Joint [%s] is of type REVOLUTE but it does not specify limits", joint.name.c_str());
        return false;
    }
    else if (joint.type == Joint::PRISMATIC)
    {
        logError("Joint [%s] is of type PRISMATIC without limits", joint.name.c_str());
        return false;
    }*/
}


bool modelFromURDF(const std::string & urdf_filename,
                         iDynTree::Model & output)
{
    std::ifstream ifs(urdf_filename.c_str());

    if( !ifs.is_open() )
    {
        std::cerr << "[ERROR] iDynTree::modelFromURDF : error opening file "
                  << urdf_filename << std::endl;
        return false;
    }

    std::string xml_string( (std::istreambuf_iterator<char>(ifs) ),
                            (std::istreambuf_iterator<char>()    ) );

    return modelFromURDFString(xml_string,output);
}

/**
 * Check if a given transform is the identity, up to the transform.
 */
bool isIdentity(const Transform & trans)
{
    Rotation rot = trans.getRotation();
    Position pos = trans.getPosition();

    for(int row=0; row < 3; row++ )
    {
        for(int col=0; col < 3; col++ )
        {
            if( row == col )
            {
                if( fabs(rot(row,col)-1) > URDFImportTol )
                {
                    return false;
                }
            }
            else
            {
                if( fabs(rot(row,col)) > URDFImportTol )
                {
                    return false;
                }
            }
        }
    }

    for(int row=0; row < 3; row++ )
    {
        if( fabs(pos(row)) > URDFImportTol )
        {
            return false;
        }
    }

    return true;
}

/**
 * Check the condition for deciding if a model has a fake base link.
 * The three conditions for a base link to be considered "fake" are:
 *  * if the base link is massless,
 *  * if the base link has only one child,
 *  * if the base link is attached to its only child with a fixed joint,
 *  * if the transform between the base link and its child is the identity.
 *
 */
bool hasFakeBaseLink(const Model& modelWithFakeLinks)
{
    LinkIndex baseLink = modelWithFakeLinks.getDefaultBaseLink();

    // First condition: base link is massless
    double mass = modelWithFakeLinks.getLink(baseLink)->getInertia().getMass();
    if( mass > URDFImportTol )
    {
        return false;
    }

    // Second condition: the base link has only one child
    if( modelWithFakeLinks.getNrOfNeighbors(baseLink) != 1 )
    {
        return false;
    }

    // Third condition: the base link is attached to its child with a fixed joint
    Neighbor neigh = modelWithFakeLinks.getNeighbor(baseLink,0);
    if( modelWithFakeLinks.getJoint(neigh.neighborJoint)->getNrOfDOFs() > 0 )
    {
        return false;
    }

    // Fourth condition: the transform between the base link and its child is the identity
    FixedJoint * pFixedJoint = (FixedJoint *) modelWithFakeLinks.getJoint(neigh.neighborJoint);
    Transform base_T_child = pFixedJoint->getTransform(baseLink,neigh.neighborLink);

    if( !isIdentity(base_T_child) )
    {
        return false;
    }

    return true;
}

/**
 * In a URDF model several links are added to the URDF file,
 * but are not proper links:
 *  * a root massless link (usually called "base_link") is
 *    added to floating base models as a workaround to an
 *    old limitation of the KDL library ( https://github.com/ros/robot_model/issues/6).
 *  * massless leaf links attached with fixed joints to real links
 *    are used to represent frames, as the URDF does not support
 *    frames attached to a link.
 *
 * This function is used to remove this "fake" links from the model after parsing.
 * \todo TODO implement additional frames for links, to parse this
 *            fake links as frames.
 *
 */
bool removeFakeLinks(const Model& modelWithFakeLinks,
                     Model& modelWithoutFakeLinks)
{
    std::set<std::string> linkToRemove;
    std::set<std::string> jointToRemove;
    std::string newDefaultBaseLink = modelWithFakeLinks.getLinkName(modelWithFakeLinks.getDefaultBaseLink());

    if( hasFakeBaseLink(modelWithFakeLinks) )
    {
        LinkIndex baseLink = modelWithFakeLinks.getDefaultBaseLink();
        linkToRemove.insert(modelWithFakeLinks.getLinkName(modelWithFakeLinks.getDefaultBaseLink()));
        JointIndex jntIndex = modelWithFakeLinks.getNeighbor(baseLink,0).neighborJoint;
        jointToRemove.insert(modelWithFakeLinks.getJointName(jntIndex));
        LinkIndex newBaseIndex =  modelWithFakeLinks.getNeighbor(baseLink,0).neighborLink;
        newDefaultBaseLink = modelWithFakeLinks.getLinkName(newBaseIndex);
    }

    // Create the new model obtained removing all the fake links (and relative joints)
    modelWithoutFakeLinks = Model();
    // Add all links, expect for the one to remove
    for(unsigned int lnk=0; lnk < modelWithFakeLinks.getNrOfLinks(); lnk++ )
    {
        std::string linkToAdd = modelWithFakeLinks.getLinkName(lnk);
        if( linkToRemove.find(linkToAdd) == linkToRemove.end() )
        {
            modelWithoutFakeLinks.addLink(linkToAdd,*modelWithFakeLinks.getLink(lnk));
        }
    }

    // Add all joints, preserving the numbering
    for(unsigned int jnt=0; jnt < modelWithFakeLinks.getNrOfJoints(); jnt++ )
    {
        std::string jointToAdd = modelWithFakeLinks.getJointName(jnt);
        if( jointToRemove.find(jointToAdd) == jointToRemove.end() )
        {
            // we need to change the link index in the new joints
            // to match the new link serialization
            IJointPtr newJoint = modelWithFakeLinks.getJoint(jnt)->clone();
            std::string firstLinkName = modelWithFakeLinks.getLinkName(newJoint->getFirstAttachedLink());
            std::string secondLinkName = modelWithFakeLinks.getLinkName(newJoint->getSecondAttachedLink());
            JointIndex  firstLinkNewIndex = modelWithoutFakeLinks.getLinkIndex(firstLinkName);
            JointIndex  secondLinkNewIndex = modelWithoutFakeLinks.getLinkIndex(secondLinkName);
            newJoint->setAttachedLinks(firstLinkNewIndex,secondLinkNewIndex);

            modelWithoutFakeLinks.addJoint(jointToAdd,newJoint);

            delete newJoint;
        }
    }

    // Set the default base link
    return modelWithoutFakeLinks.setDefaultBaseLink(modelWithoutFakeLinks.getLinkIndex(newDefaultBaseLink));
}

/**
 * Helper function of modelFromURDFString , used to cleanup the
 * fixed joints temporary vector in case of error.
 *
 */
void cleanupFixedJoints(std::vector<IJointPtr> & fixedJoints)
{
    for(int i=0; i < fixedJoints.size(); i++)
    {
        if( fixedJoints[i] )
        {
            delete fixedJoints[i];
            fixedJoints[i] = 0;
        }
    }
}

bool modelFromURDFString(const std::string& urdf_string,
                               iDynTree::Model& model)
{
    bool ok = true;

    // clear the input model
    Model rawModel = Model();

    TiXmlDocument urdfXml;
    urdfXml.Parse(urdf_string.c_str());

    TiXmlElement* robotXml = urdfXml.FirstChildElement("robot");

    // parse all links
    for (TiXmlElement* link_xml = robotXml->FirstChildElement("link");
         link_xml; link_xml = link_xml->NextSiblingElement("link") )
    {
        Link link;
        std::string linkName;
        ok = ok && linkFromURDFXML(link_xml,link,linkName);
        if( !ok )
        {
            model = Model();
            return false;
        }

        LinkIndex newLinkIndex = rawModel.addLink(linkName,link);

        if( newLinkIndex == LINK_INVALID_INDEX )
        {
            model = Model();
            return false;
        }
    }

    // parse all joints, saving a set of parents and childs
    std::set<std::string> parents;
    std::set<std::string> childs;
    // we parse the fixed joints separatly, so we can add them
    // all after the joints with a non-zero dofs . This is necessary
    // for backward compatibility with KDL-based software
    std::vector<IJointPtr> fixedJoints;
    std::vector<std::string> fixedJointNames;
    for (TiXmlElement* joint_xml = robotXml->FirstChildElement("joint");
         joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
    {
        IJointPtr joint=0;
        std::string jointName;
        std::string parentLinkName;
        std::string childLinkName;
        std::string jointType;

        ok = ok && jointFromURDFXML(rawModel,joint_xml,joint,jointName,parentLinkName,childLinkName,jointType);

        // save parent and child in a set
        parents.insert(parentLinkName);
        childs.insert(childLinkName);

        if( !ok )
        {
            cleanupFixedJoints(fixedJoints);
            delete joint;
            model = Model();
            return false;
        }

        assert(joint->getFirstAttachedLink() != joint->getSecondAttachedLink());

        // All joints expect the fixed one are immediatly add to the model
        if( jointType != "fixed" )
        {
            JointIndex newJointIndex = rawModel.addJoint(jointName,joint);

            delete joint;

            if( newJointIndex == JOINT_INVALID_INDEX )
            {
                cleanupFixedJoints(fixedJoints);
                model = Model();
                return false;
            }
        }
        else
        {
            // fixed joints are stored separatly and added after the non-zero dofs joints
            fixedJoints.push_back(joint);
            fixedJointNames.push_back(jointName);
        }

    }

    // Adding all the fixed joint in the end
    for(int i=0; i < fixedJoints.size(); i++)
    {
        assert( fixedJoints.size() == fixedJointNames.size() );
        JointIndex newJointIndex = rawModel.addJoint(fixedJointNames[i],fixedJoints[i]);

        delete fixedJoints[i];
        fixedJoints[i] = 0;

        if( newJointIndex == JOINT_INVALID_INDEX )
        {
            cleanupFixedJoints(fixedJoints);
            model = Model();
            return false;
        }
    }


    // Get root
    std::vector<std::string> rootCandidates;
    for(unsigned int lnk=0; lnk < rawModel.getNrOfLinks(); lnk++ )
    {
        std::string linkName = rawModel.getLinkName(lnk);

        if( childs.find(linkName) == childs.end() )
        {
            rootCandidates.push_back(linkName);
        }
    }

    if( rootCandidates.size() == 0 )
    {
        reportError("","modelFromURDFString","No root link found in URDF string");
        model = Model();
        return false;
    }

    if( rootCandidates.size() >= 2 )
    {
        reportError("","modelFromURDFString","Multiple root links found in URDF string");
        model = Model();
        return false;
    }

    // set the default root in the model
    rawModel.setDefaultBaseLink(rawModel.getLinkIndex(rootCandidates[0]));

    // Remove fake links
    ok = ok && removeFakeLinks(rawModel,model);

    return ok;
}

}

