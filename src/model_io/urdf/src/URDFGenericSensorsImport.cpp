/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include <iDynTree/ModelIO/URDFGenericSensorsImport.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/URDFModelImport.h>


#include <iDynTree/Core/Transform.h>

#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/SixAxisFTSensor.h>
#include <iDynTree/Sensors/AccelerometerSensor.h>
#include <iDynTree/Sensors/GyroscopeSensor.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Indeces.h>

#include <tinyxml.h>

#include <algorithm>
#include <iostream>
#include <fstream>

namespace iDynTree
{

std::vector<std::string> &splitString(const std::string &s, std::vector<std::string> &elems)
{
    std::stringstream ss(s);
    std::string item;
    while (ss >> item)
    {
        elems.push_back(item);
    }
    return elems;
}

bool findJointParentsAndChild(TiXmlElement* robotXml,
                              const std::string queriedJointName,
                              std::string & parentLinkName,
                              std::string & childLinkName)
{
    for (TiXmlElement* joint_xml = robotXml->FirstChildElement("joint");
         joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
    {
        std::string jointName;

        const char *name = joint_xml->Attribute("name");
        if (!name)
        {
            reportError("","findJointParentsAndChild","unnamed joint found");
            return false;
        }
        jointName = name;

        if( jointName == queriedJointName )
        {
            TiXmlElement *parent_xml = joint_xml->FirstChildElement("parent");
            if (parent_xml)
            {
                const char *pname = parent_xml->Attribute("link");
                if (!pname)
                {
                    std::string errStr = "No parent specified for joint " + jointName;
                    reportError("","findJointParentsAndChild",errStr.c_str());
                    return false;
                }
                else
                {
                    parentLinkName = std::string(pname);
                }
            }
            else
            {
                std::string errStr = "No parent specified for joint " + jointName;
                reportError("","findJointParentsAndChild",errStr.c_str());
                return false;
            }

            // Get Child Link
            TiXmlElement *child_xml = joint_xml->FirstChildElement("child");
            if (child_xml)
            {
                const char *pname = child_xml->Attribute("link");
                if (!pname)
                {
                    std::string errStr = "No child specified for joint " + jointName;
                    reportError("","findJointParentsAndChild",errStr.c_str());
                    return false;
                }
                else
                {
                    childLinkName = std::string(pname);
                }
            }
            else
            {
                std::string errStr = "No child specified for joint " + jointName;
                reportError("","findJointParentsAndChild",errStr.c_str());
                return false;
            }

            return true;
        }
    }

    std::string errStr = "Joint " + queriedJointName + " not found in the model.";
    reportError("","findJointParentsAndChild",errStr.c_str());
    return false;
}

bool sensorsFromURDFString(const std::string& urdfXml,
                           const Model & model,
                           SensorsList & outputSensors)
{
    SensorsList newSensors;

    bool parsingSuccessful = true;

    /* parse custon  urdf */
    TiXmlDocument tiUrdfXml;
    tiUrdfXml.Parse(urdfXml.c_str());

    TiXmlElement* robotXml = tiUrdfXml.FirstChildElement("robot");

    // Several sensor types have been proposed for use in URDF,
    // see http://wiki.ros.org/urdf/XML/sensor/proposals ,
    // but few of them (the one more relevant to dynamics) are supported by iDynTree
    // To avoid printing too many warnings, a warning about the ignored sensor
    // is printed only if the sensor has a type that is not on the following list
    // In this way we can print a warning to catch eventual typos, but avoid flooding
    // the user with meaningless warnings if he loads (for example) a model full of cameras
    std::vector<std::string> sensorTypesUsedInURDFButNotSupportedInIDynTree;
    sensorTypesUsedInURDFButNotSupportedInIDynTree.push_back("camera");
    sensorTypesUsedInURDFButNotSupportedInIDynTree.push_back("ray");
    sensorTypesUsedInURDFButNotSupportedInIDynTree.push_back("imu");
    sensorTypesUsedInURDFButNotSupportedInIDynTree.push_back("magnetometer");
    sensorTypesUsedInURDFButNotSupportedInIDynTree.push_back("gps");
    sensorTypesUsedInURDFButNotSupportedInIDynTree.push_back("contact");
    sensorTypesUsedInURDFButNotSupportedInIDynTree.push_back("sonar");
    sensorTypesUsedInURDFButNotSupportedInIDynTree.push_back("rfidtag");
    sensorTypesUsedInURDFButNotSupportedInIDynTree.push_back("rfid");


    // Get all the sensors elements of robot element
    for (TiXmlElement* sensorXml = robotXml->FirstChildElement("sensor");
         sensorXml; sensorXml = sensorXml->NextSiblingElement("sensor"))
    {
        if( sensorXml->Attribute("name")==NULL ||
            sensorXml->Attribute("type")==NULL )
        {
            std::cerr << "[ERROR] Sensor name or type specified incorrectly." << std::endl;
            parsingSuccessful = false;
            return false;
        }

        // Parse sensors name
        std::string  sensorName(sensorXml->Attribute("name"));

        // Parse sensor type
        std::string sensorType(sensorXml->Attribute("type"));

        iDynTree::SensorType type;



        if(sensorType.compare("accelerometer") == 0)
        {
            type = ACCELEROMETER;
        }
        else if(sensorType.compare("gyroscope") == 0)
        {
            type = GYROSCOPE;
        }
        else if(sensorType.compare("force_torque") == 0)
        {
            type = SIX_AXIS_FORCE_TORQUE;
        }
        else
        {
            // Print the warning about ignored sensors only if the sensor type is not on the list (to catch typos)
            if( std::find(sensorTypesUsedInURDFButNotSupportedInIDynTree.begin(),sensorTypesUsedInURDFButNotSupportedInIDynTree.end(), sensorType)
                     == sensorTypesUsedInURDFButNotSupportedInIDynTree.end() )
            {
                std::string errString = "Specified sensor type " + sensorType + " of sensor with name " + sensorName +" is not recognised, this sensor was ignored by the iDynTree sensor parser.";
                reportWarning("","sensorsFromURDFString",errString.c_str());
            }
            continue;
        }

        // Parse origin element
        Transform link_H_pose;
        TiXmlElement* originTag = sensorXml->FirstChildElement("origin");
        if( originTag )
        {
            std::string  rpyText(originTag->Attribute("rpy"));
            std::string  xyzText(originTag->Attribute("xyz"));

            std::vector<std::string> rpyElems;
            std::vector<std::string> xyzElems;
            splitString(rpyText,rpyElems);
            splitString(xyzText,xyzElems);

            if( rpyElems.size() != 3 && xyzElems.size() !=3 )
            {
               std::cerr<<"[ERROR] Pose attribute for sensor " << sensorName << " specified incorrectly, parsing failed.";
               parsingSuccessful = false;
               return false;
            }
            double roll  = atof(rpyElems[0].c_str());
            double pitch = atof(rpyElems[1].c_str());
            double yaw   = atof(rpyElems[2].c_str());
            iDynTree::Rotation rot = iDynTree::Rotation::RPY(roll,pitch,yaw);
            double x  = atof(xyzElems[0].c_str());
            double y = atof(xyzElems[1].c_str());
            double z   = atof(xyzElems[2].c_str());
            iDynTree::Position pos(x,y,z);
            link_H_pose = iDynTree::Transform(rot,pos);
        }
        else
        {
            // default value
            link_H_pose = iDynTree::Transform::Identity();
        }

        // Parse parent element
        TiXmlElement* parent = sensorXml->FirstChildElement("parent");

        std::string parentLink, parentJoint;
        JointIndex parentJointIndex = JOINT_INVALID_INDEX;
        LinkIndex  parentLinkIndex  = LINK_INVALID_INDEX;

        if(!parent)
        {
            std::cerr<< "[ERROR] parent element not found in sensor " << sensorName <<
                        ", parsing failed." << std::endl;
            parsingSuccessful = false;
            return false;
        }

        if(parent->Attribute("link")!= NULL)
        {
            parentLink = std::string(parent->Attribute("link"));
            parentLinkIndex = model.getLinkIndex(parentLink);

            if( parentLinkIndex == LINK_INVALID_INDEX )
            {
               std::cerr<< "[ERROR] sensor " << sensorName
                         << " has parent link " << parentLink
                         << " but the link is not found on the model, parsing failed." << std::endl;
                parsingSuccessful = false;
                return false;
            }

            if( type == SIX_AXIS_FORCE_TORQUE )
            {
                std::cerr<< "[ERROR] sensor " << sensorName
                         << " of type force_torque cannot be child of a link "
                         << ", parsing failed." << std::endl;
                parsingSuccessful = false;
                return false;
            }
        }
        else if(parent->Attribute("joint")!=NULL)
        {
            parentJoint = std::string(parent->Attribute("joint"));

            parentJointIndex = model.getJointIndex(parentJoint);

            if( parentJointIndex == JOINT_INVALID_INDEX )
            {
               std::cerr<< "[ERROR] sensor " << sensorName
                         << " has parent joint " << parentJoint
                         << " but the joint is not found on the model, parsing failed." << std::endl;
                parsingSuccessful = false;
                return false;
            }

            if( type == GYROSCOPE )
            {
                std::cerr<< "[ERROR] sensor " << sensorName
                         << " of type gyroscope cannot be child of a joint "
                         << ", parsing failed." << std::endl;
                parsingSuccessful = false;
                return false;
            }

            if( type == ACCELEROMETER )
            {
                std::cerr<< "[ERROR] sensor " << sensorName
                         << " of type accelerometer cannot be child of a joint "
                         << ", parsing failed." << std::endl;
                parsingSuccessful = false;
                return false;
            }
        }
        else
        {
            std::cerr << "[ERROR] Link or Joint attribute for parent element not found in sensor "
                      << sensorName << ", parsing failed." << std::endl;
            parsingSuccessful = false;
            return false;
        }

        // Actually allocate the sensors
        if( type == SIX_AXIS_FORCE_TORQUE )
        {
            // parse frame
            enum { PARENT_LINK_FRAME ,
                   CHILD_LINK_FRAME  ,
                    SENSOR_FRAME } frame_type;

            enum { PARENT_TO_CHILD,
                   CHILD_TO_PARENT } measure_direction_type;

            TiXmlElement* force_torque = sensorXml->FirstChildElement("force_torque");

            if( !force_torque )
            {
                std::cerr<< "[ERROR] sensor " << sensorName
                         << " of type force_torque is missing force_torque element "
                         << ", parsing failed." << std::endl;
                parsingSuccessful = false;
                return false;
            }

            TiXmlElement* frame_el = force_torque->FirstChildElement("frame");
            TiXmlElement* measure_direction_el = force_torque->FirstChildElement("measure_direction");

            // parse frame
            if( !frame_el || !measure_direction_el )
            {
                std::cerr<< "[ERROR] sensor " << sensorName
                         << " of type force_torque is missing frame or measure_direction element "
                         << ", parsing failed." << std::endl;
                parsingSuccessful = false;
                return false;
            }

            std::string frame = frame_el->GetText();
            if( frame == "child" )
            {
                frame_type = CHILD_LINK_FRAME;
            }
            else if( frame == "parent" )
            {
                frame_type = PARENT_LINK_FRAME;
            }
            else if( frame == "sensor" )
            {
                frame_type = SENSOR_FRAME;
            }
            else
            {
                std::cerr<< "[ERROR] sensor " << sensorName
                         << " of type force_torque has unexpected frame content " << frame
                         << ", parsing failed." << std::endl;
                parsingSuccessful = false;
                return false;
            }

            std::string measure_direction = measure_direction_el->GetText();
            if( measure_direction == "parent_to_child" )
            {
                measure_direction_type = PARENT_TO_CHILD;
            }
            else if( measure_direction == "child_to_parent" )
            {
                measure_direction_type = CHILD_TO_PARENT;
            }
            else
            {
                std::cerr<< "[ERROR] sensor " << sensorName
                         << " of type force_torque has unexpected measure_direction content " << measure_direction
                         << ", parsing failed." << std::endl;
                parsingSuccessful = false;
                return false;
            }

            // Get parent and child links of the joint to which the FT sensor is attached
            std::string parentLinkName, childLinkName;
            bool ok = findJointParentsAndChild(robotXml,parentJoint,parentLinkName,childLinkName);

            if( !ok )
            {
                parsingSuccessful = false;
                return false;
            }

            LinkIndex parentLinkIndex = model.getLinkIndex(parentLinkName);
            LinkIndex childLinkIndex  = model.getLinkIndex(childLinkName);

            if( parentLinkIndex == LINK_INVALID_INDEX ||
                childLinkIndex  == LINK_INVALID_INDEX )
            {
                std::cerr<< "[ERROR] link " << parentLinkName << " or " << childLinkName
                         << " not found when parsing sensor " << sensorName
                         << ", parsing failed." << std::endl;
                parsingSuccessful = false;
                return false;
            }

            SixAxisForceTorqueSensor * sensor = new SixAxisForceTorqueSensor();
            sensor->setName(sensorName);
            sensor->setParentJoint(parentJoint);
            sensor->setParentJointIndex(parentLinkIndex);
            if( measure_direction_type == PARENT_TO_CHILD )
            {
                sensor->setAppliedWrenchLink(childLinkIndex);
            }
            else
            {
                sensor->setAppliedWrenchLink(parentLinkIndex);
            }

            // The transform tag is parsed using the Gazebo convention
            // For now we assume that the six axis ft sensor is attached to a
            // fixed junction. Hence the first/second link to sensor transforms
            // are fixed are given by the frame option
            iDynTree::Transform parent_link_H_child_link = model.getJoint(parentJointIndex)->getRestTransform(parentLinkIndex,childLinkIndex);
            iDynTree::Transform child_link_H_sensor = link_H_pose;

            if( frame_type == PARENT_LINK_FRAME )
            {
                sensor->setFirstLinkSensorTransform(parentLinkIndex,iDynTree::Transform::Identity());
                sensor->setSecondLinkSensorTransform(childLinkIndex,parent_link_H_child_link.inverse());
            }
            else if( frame_type == CHILD_LINK_FRAME )
            {
                sensor->setFirstLinkSensorTransform(parentLinkIndex,parent_link_H_child_link);
                sensor->setSecondLinkSensorTransform(childLinkIndex,iDynTree::Transform::Identity());
            }
            else
            {
                assert( frame_type == SENSOR_FRAME );
                sensor->setFirstLinkSensorTransform(parentLinkIndex,parent_link_H_child_link*child_link_H_sensor);
                sensor->setSecondLinkSensorTransform(childLinkIndex,child_link_H_sensor);
            }

            sensor->setFirstLinkName(parentLinkName);
            sensor->setSecondLinkName(childLinkName);

            newSensors.addSensor(*sensor);
            delete sensor;
        }
        else if( type == GYROSCOPE )
        {
            GyroscopeSensor * sensor = new GyroscopeSensor();
            sensor->setLinkSensorTransform(link_H_pose);
            sensor->setName(sensorName);
            sensor->setParentLink(parentLink);
            sensor->setParentLinkIndex(parentLinkIndex);
            newSensors.addSensor(*sensor);
            delete sensor;
        }
        else if( type == ACCELEROMETER )
        {
            AccelerometerSensor * sensor = new AccelerometerSensor();
            sensor->setLinkSensorTransform(link_H_pose);
            sensor->setName(sensorName);
            sensor->setParentLink(parentLink);
            sensor->setParentLinkIndex(parentLinkIndex);
            newSensors.addSensor(*sensor);
            delete sensor;
        }
        else
        {
           std::cerr<< "[ERROR] sensor " << sensorName
                         << " of unknown type "
                         << ", parsing failed." << std::endl;
           parsingSuccessful = false;
           return false;
        }

    }

    // If the parsing was successful,
    // copy the obtained sensors to the output structure
    if( parsingSuccessful )
    {
        outputSensors = newSensors;
    }

    return parsingSuccessful;
}


bool sensorsFromURDF(const std::string & urdfFilename,
                     const iDynTree::Model & undirectedTree,
                     SensorsList & sensors)
{
    std::ifstream ifs(urdfFilename.c_str());
    std::string xmlString( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );

    return sensorsFromURDFString(xmlString,undirectedTree,sensors);
}

bool sensorsFromURDF(const std::string & urdf_filename,
                     iDynTree::SensorsList & output)
{
    iDynTree::Model model;
    bool ok = modelFromURDF(urdf_filename,model);

    if( !ok )
    {
        std::cerr << "[ERROR] iDynTree::sensorsListFromURDF : error in loading urdf "
                  << urdf_filename << std::endl;
        return false;
    }

    ok = sensorsFromURDF(urdf_filename,model,output);

    return ok;
}

bool sensorsFromURDFString(const std::string& urdf_string,
                           iDynTree::SensorsList& output)
{
    iDynTree::Model model;
    bool ok = modelFromURDFString(urdf_string,model);

    if( !ok )
    {
        std::cerr << "[ERROR] iDynTree::sensorsListFromURDFString : error in loading urdf  string "
                  << urdf_string << std::endl;
        return false;
    }


    ok = sensorsFromURDFString(urdf_string,model,output);

    return ok;
}



}

