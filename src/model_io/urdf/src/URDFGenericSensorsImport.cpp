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

#include <iostream>
#include <fstream>

namespace iDynTree
{

struct GenericSensorData
{
    enum { LINK, JOINT} parentObject;
    std::string parentObjectName;
    std::string sensorName;
    SensorType sensorType;
    iDynTree::Transform sensorPose;
    unsigned int updateRate;
};

typedef GenericSensorData AccelerometerData;
typedef GenericSensorData GyroscopeData;

struct GenericFTSensorData : GenericSensorData
{
     enum { PARENT_LINK_FRAME ,
         CHILD_LINK_FRAME  ,
         SENSOR_FRAME } frame;
     enum { PARENT_TO_CHILD,
         CHILD_TO_PARENT }
         measure_direction;
};

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

bool genericSensorsFromUrdfString(const std::string& urdfXml, std::vector<GenericSensorData> & genericSensors)
{
    genericSensors.resize(0);

    bool returnVal = true;

    std::cerr << urdfXml << std::endl;

    /* parse sdf force_torque extension to urdf */
    TiXmlDocument tiUrdfXml;
    tiUrdfXml.Parse(urdfXml.c_str());

    TiXmlElement* robotXml = tiUrdfXml.FirstChildElement("robot");

    // Get all SDF extension elements and search for the sensors
    for (TiXmlElement* sensorXml = robotXml->FirstChildElement("sensor");
         sensorXml; sensorXml = sensorXml->NextSiblingElement("sensor"))
    {
        GenericSensorData newGenericSensor;

        if(sensorXml->Attribute("name")==NULL || sensorXml->Attribute("type")==NULL)
        {
            std::cerr<<"Sensor name or type specified incorrectly \n";
            returnVal = false;
            break;
        }
        std::string  sensorName(sensorXml->Attribute("name"));

        newGenericSensor.sensorName = sensorName;
        std::string sensorType(sensorXml->Attribute("type"));

        if(sensorType.compare("accelerometer") == 0)
        {
            newGenericSensor.sensorType = ACCELEROMETER;
        }
        else if(sensorType.compare("gyroscope") == 0)
        {
            newGenericSensor.sensorType = GYROSCOPE;
        }else if(sensorType.compare("force_torque") == 0)
        {
            newGenericSensor.sensorType = SIX_AXIS_FORCE_TORQUE;
        }else
        {
            std::cerr<<"Specified sensor type "<<sensorType<<" is not recognised\n";
            returnVal = false;
            break;
        }

        TiXmlElement* updateRate = sensorXml->FirstChildElement("update_rate");
        if(updateRate)
        {
            std::string updateRateString = updateRate->GetText();
            newGenericSensor.updateRate =atoi(updateRateString.c_str());
        }
        else
        {
            // default value
            newGenericSensor.updateRate =100;
        }
        TiXmlElement* parent = sensorXml->FirstChildElement("parent");

        std::string linkOption, jointOption;

        if(!parent)
        {
            std::cerr<<"parent object not found \n";
            returnVal = false;
            break;
        }
        if(parent->Attribute("link")!= NULL)
        {
            linkOption = std::string(parent->Attribute("link"));
        }
        else if(parent->Attribute("joint")!=NULL)
        {
            jointOption = std::string(parent->Attribute("joint"));
        }
        else
        {
                std::cerr<<"link or joint attribute for parent object not found \n";
                returnVal = false;
                break;
        }

        if(!linkOption.empty())
        {
              newGenericSensor.parentObject = iDynTree::GenericSensorData::LINK;
              newGenericSensor.parentObjectName = linkOption;
        }
        else
        {
              newGenericSensor.parentObject = iDynTree::GenericSensorData::JOINT;
              newGenericSensor.parentObjectName = jointOption;
        }

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
                std::cerr<<"Pose attribute for sensor specified incorrectly";
               returnVal = false;
            }
            double roll  = atof(rpyElems[0].c_str());
            double pitch = atof(rpyElems[1].c_str());
            double yaw   = atof(rpyElems[2].c_str());
            iDynTree::Rotation rot = iDynTree::Rotation::RPY(roll,pitch,yaw);
            double x  = atof(xyzElems[0].c_str());
            double y = atof(xyzElems[1].c_str());
            double z   = atof(xyzElems[2].c_str());
            iDynTree::Position pos(x,y,z);
            newGenericSensor.sensorPose = iDynTree::Transform(rot,pos);
        }
        else
        {
            // default value
            newGenericSensor.sensorPose = iDynTree::Transform::Identity();
        }

        genericSensors.push_back(newGenericSensor);

    }

    return returnVal;
}


bool genericSensorsFromUrdfFile(const std::string& file, std::vector<GenericSensorData> & generic_sensors)
{
    std::ifstream ifs(file.c_str());
    std::string xml_string( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );

    return genericSensorsFromUrdfString(xml_string,generic_sensors);
}

bool genericSensorsListFromURDFString(iDynTree::Model & undirectedTree,
                                      std::string urdfString,
                                      SensorsList& sensors)
{
    SensorsList sensorsTree;
    std::vector<iDynTree::GenericSensorData> genericSensors;

    bool ok = iDynTree::genericSensorsFromUrdfString(urdfString,genericSensors);

    if( !ok )
    {
        std::cerr << "[ERROR] loading generic sensors information from URDF file" << std::endl;
        return ok;
    }

    for(size_t genSensItr = 0; genSensItr < genericSensors.size(); genSensItr++ )
    {
        iDynTree::Sensor *newSensor = 0;
        switch(genericSensors[genSensItr].sensorType)
        {
            case iDynTree::SIX_AXIS_FORCE_TORQUE :
                {
                    iDynTree::SixAxisForceTorqueSensor *newFTSensor = new(iDynTree::SixAxisForceTorqueSensor);
                    newSensor = (Sensor *)newFTSensor;
                }
                break;
            case iDynTree::ACCELEROMETER :
                {
                    iDynTree::AccelerometerSensor *newAccelerometerSens = new(iDynTree::AccelerometerSensor);
                    newAccelerometerSens->setLinkSensorTransform(genericSensors[genSensItr].sensorPose);
                    newSensor = (Sensor *)newAccelerometerSens;
                }
                break;
            case iDynTree::GYROSCOPE :
                {
                    iDynTree::GyroscopeSensor *newGyroscopeSens = new(iDynTree::GyroscopeSensor);
                    newGyroscopeSens->setLinkSensorTransform(genericSensors[genSensItr].sensorPose);
                    newSensor = (Sensor*)newGyroscopeSens;
                }
                break;
        }


        // setting the parent object (junction / link) names
        newSensor->setName(genericSensors[genSensItr].sensorName);
        newSensor->setParent(genericSensors[genSensItr].parentObjectName);

       //setting the parent index by obtaining from the undirected tree
       // first version only for links not junctions

       if(genericSensors[genSensItr].parentObject == iDynTree::GenericSensorData::LINK)
       {
           // TODO \todo Actually report and error to the user
           iDynTree::LinkIndex parentLinkIndex = undirectedTree.getLinkIndex(newSensor->getParent());

           if( parentLinkIndex == LINK_INVALID_INDEX )
           {
               std::cerr << "[ERROR] impossible to find link " << newSensor->getParent() << " in the model" << std::endl;
               return false;
           }

           newSensor->setParentIndex(parentLinkIndex);
       }
       else
       {
           std::cerr<<"[ERROR] obtaining parent junction index not yet implemented, initial version only for link based sensors like accelerometer and gyroscopes\n";
           return false;
       }

       int sensorIndex = sensorsTree.addSensor(*newSensor);
       //since cloning is completed we can delete to free the sensor memory allocation
       delete(newSensor );
    }

    sensors = sensorsTree;
    return ok;
}


bool genericSensorsListFromURDF(iDynTree::Model & undirectedTree,
                                std::string urdfFilename,
                                SensorsList & sensors)
{
    std::ifstream ifs(urdfFilename.c_str());
    std::string xmlString( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );

    return genericSensorsListFromURDFString(undirectedTree,xmlString,sensors);
}

bool genericSensorsListFromURDF(const std::string & urdf_filename,
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

    ok = genericSensorsListFromURDF(model,urdf_filename,output);

    return ok;
}

bool genericSensorsListFromURDFString(const std::string& urdf_string,
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


    ok = genericSensorsListFromURDFString(model,urdf_string,output);

    return ok;
}



}

