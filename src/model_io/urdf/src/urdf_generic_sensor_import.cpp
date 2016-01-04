/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Fondazione Istituto Italiano di Tecnologia
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

/* Author: Naveen Kuppuswamy */


#include <iDynTree/Core/Transform.h>

#include <kdl_codyco/undirectedtree.hpp>

#include <kdl_codyco/KDLConversions.h>

#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/SixAxisFTSensor.h>
#include <iDynTree/Sensors/Accelerometer.h>
#include <iDynTree/Sensors/Gyroscope.h>


#include <fstream>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <tinyxml.h>
#include <iDynTree/ModelIO/impl/urdf_sensor_import.hpp>
#include <iDynTree/ModelIO/impl/urdf_generic_sensor_import.hpp>
using namespace std;
using namespace KDL;

namespace iDynTree{


bool genericSensorsFromUrdfFile(const std::string& file, std::vector<GenericSensorData> & generic_sensors)
{
    ifstream ifs(file.c_str());
    std::string xml_string( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );

    return genericSensorsFromUrdfString(xml_string,generic_sensors);
}

bool genericSensorsFromUrdfString(const std::string& urdfXml, std::vector<GenericSensorData> & genericSensors)
{
    genericSensors.resize(0);

    bool returnVal = true;

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
        }else if(sensorType.compare("gyroscope") == 0)
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
            split(rpyText,rpyElems);
            split(xyzText,xyzElems);

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
iDynTree::SensorsList genericSensorsListFromURDF(KDL::CoDyCo::UndirectedTree & undirectedTree,
                                          std::string urdfFilename)
{
    ifstream ifs(urdfFilename.c_str());
    std::string xmlString( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );

    return genericSensorsListFromURDFString(undirectedTree,xmlString);
}

iDynTree::SensorsList genericSensorsListFromURDFString(KDL::CoDyCo::UndirectedTree & undirectedTree,
                                                std::string urdfString)
{
    iDynTree::SensorsList sensorsTree;
    std::vector<iDynTree::GenericSensorData> genericSensors;

    bool ok = iDynTree::genericSensorsFromUrdfString(urdfString,genericSensors);

    if( !ok )
    {
        std::cerr << "Error in loading generic sensors information from URDF file" << std::endl;
        // Return and empty sensors Tree object
        return sensorsTree;
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
                    iDynTree::Accelerometer *newAccelerometerSens = new(iDynTree::Accelerometer);
                    newAccelerometerSens->setLinkSensorTransform(genericSensors[genSensItr].sensorPose);
                    newSensor = (Sensor *)newAccelerometerSens;
                }
                break;
            case iDynTree::GYROSCOPE :
                {
                    iDynTree::Gyroscope *newGyroscopeSens = new(iDynTree::Gyroscope);
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
           KDL::CoDyCo::LinkMap::const_iterator link_it= undirectedTree.getLink(newSensor->getParent());
           newSensor->setParentIndex(link_it->getLinkIndex());
       }
       else
       {
           std::cerr<<"obtaining parent junction index not yet implemented, initial version only for link based sensors like accelerometer and gyroscopes\n";
       }
       sensorsTree.addSensor(*newSensor);
       //since cloning is completed we can delete to free the sensor memory allocation
       delete(newSensor );
    }

    return sensorsTree;
}

}

