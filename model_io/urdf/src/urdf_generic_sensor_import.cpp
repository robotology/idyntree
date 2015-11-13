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

#include <iDynTree/ModelIO/impl/urdf_generic_sensor_import.hpp>
#include <iDynTree/ModelIO/impl/urdf_sensor_import.hpp>
#include <iDynTree/Core/Transform.h>
#include <kdl_codyco/undirectedtree.hpp>

#include <kdl_codyco/KDLConversions.h>

#include <iDynTree/Sensors/Sensors.hpp>
#include <iDynTree/Sensors/SixAxisFTSensor.hpp>
#include <iDynTree/Sensors/Accelerometer.hpp>
#include <iDynTree/Sensors/Gyroscope.hpp>


#include <fstream>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <tinyxml.h>

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
        newGenericSensor.sensorName = sensorXml->Attribute("name");  
        std::string sensorType = sensorXml->Attribute("type");
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
            returnVal = false;
                  
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
        TiXmlElement* link = sensorXml->FirstChildElement("link");
        TiXmlElement* joint = sensorXml->FirstChildElement("joint");
        if(parent && (link || joint))
        {
            std::string objectName;
            if(link)
            {
                newGenericSensor.parentObject = iDynTree::GenericSensorData::LINK;
                objectName = link->GetText();
            }
            else
            {
               newGenericSensor.parentObject = iDynTree::GenericSensorData::JOINT;
               objectName = joint->GetText();
            }
            newGenericSensor.parentObjectName = objectName;
                                                    
         }
         else
            returnVal = false;
                  
         TiXmlElement* poseTag = sensorXml->FirstChildElement("pose");
         if( poseTag )
         {
            std::string  poseText = poseTag->GetText();
            std::vector<std::string> poseElems;
            split(poseText,poseElems);
            if( poseElems.size() != 6 )
            {
               returnVal = false;
            }
            double roll  = atof(poseElems[3].c_str());
            double pitch = atof(poseElems[4].c_str());
            double yaw   = atof(poseElems[5].c_str());
            iDynTree::Rotation rot = iDynTree::Rotation::RPY(roll,pitch,yaw);
            double x  = atof(poseElems[0].c_str());
            double y = atof(poseElems[1].c_str());
            double z   = atof(poseElems[2].c_str());
            iDynTree::Position pos(x,y,z);
            newGenericSensor.sensorPose = iDynTree::Transform(rot,pos); 
        }
        else
        {
                // default value
            newGenericSensor.sensorPose = iDynTree::Transform();
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
    }
    
    for(int genSensItr = 0; genSensItr < genericSensors.size(); genSensItr++ )
    {
        iDynTree::Sensor *newSensor;
        switch(genericSensors[genSensItr].sensorType)
        {
            case iDynTree::SIX_AXIS_FORCE_TORQUE : 
                {
                    iDynTree::SixAxisForceTorqueSensor newSixAxisFTsens;
                    newSensor = &newSixAxisFTsens;                
                }
                break;
            case iDynTree::ACCELEROMETER : 
                {
                    iDynTree::Accelerometer newAccelerometerSens;
                    newAccelerometerSens.setLinkSensorTransform(genericSensors[genSensItr].sensorPose);
                    newSensor = &newAccelerometerSens;                
                }
                break;
            case iDynTree::GYROSCOPE :
                {   
                    iDynTree::Gyroscope newGyroscopeSens;
                    newGyroscopeSens.setLinkSensorTransform(genericSensors[genSensItr].sensorPose);
                    newSensor = &newGyroscopeSens;
                }
                break;
                
        }
        
        newSensor->setName(genericSensors[genSensItr].sensorName);
        newSensor->setParent(genericSensors[genSensItr].parentObjectName);
        
//         
// 
//         // Convert the information in the FTSensorData format in
//         // a series of SixAxisForceTorqueSensor objects, using the
//         // serialization provided in the undirected_tree object
//         new_sens->setName(generic_sensors[gen_sens_itr].sensor_name);
// 
//         new_sens.setParent(generic_sensors[gen_sens_itr].reference_joint);
// 
//         KDL::CoDyCo::JunctionMap::const_iterator junct_it
//             = undirected_tree.getJunction(generic_sensors[gen_sens_itr].reference_joint);
// 
//         new_sens.setParentIndex(junct_it->getJunctionIndex());
// 
//         int parent_link = junct_it->getParentLink()->getLinkIndex();
//         int child_link = junct_it->getChildLink()->getLinkIndex();
//         std::string parent_link_name = junct_it->getParentLink()->getName();
//         std::string child_link_name = junct_it->getChildLink()->getName();
// 
//         KDL::Frame parent_link_H_child_link = junct_it->pose(0.0,false);
//         KDL::Frame child_link_H_sensor = generic_sensors[gen_sens_itr].sensor_pose;
// 
//         // For now we assume that the six axis ft sensor is attached to a
//         // fixed junction. Hence the first/second link to sensor transforms
//         // are fixed are given by the frame option
//         if( ft_sensors[ft_sens].frame == iDynTree::FTSensorData::PARENT_LINK_FRAME )
//         {
//             new_sens.setFirstLinkSensorTransform(parent_link,iDynTree::Transform());
//             new_sens.setSecondLinkSensorTransform(child_link,iDynTree::ToiDynTree(parent_link_H_child_link.Inverse()));
//         }
//         else if( ft_sensors[ft_sens].frame == iDynTree::FTSensorData::CHILD_LINK_FRAME )
//         {
//             new_sens.setFirstLinkSensorTransform(parent_link,iDynTree::ToiDynTree(parent_link_H_child_link));
//             new_sens.setSecondLinkSensorTransform(child_link,iDynTree::Transform());
//         }
//         else
//         {
//             assert( ft_sensors[ft_sens].frame == iDynTree::FTSensorData::SENSOR_FRAME );
//             new_sens.setFirstLinkSensorTransform(parent_link,iDynTree::ToiDynTree(parent_link_H_child_link*child_link_H_sensor));
//             new_sens.setSecondLinkSensorTransform(child_link,iDynTree::ToiDynTree(child_link_H_sensor));
//         }
// 
//         //set names
//         new_sens.setFirstLinkName(parent_link_name);
//         new_sens.setSecondLinkName(child_link_name);
// 
//         if( ft_sensors[ft_sens].measure_direction == iDynTree::FTSensorData::CHILD_TO_PARENT )
//         {
//             new_sens.setAppliedWrenchLink(parent_link);
//         }
//         else
//         {
//             assert( ft_sensors[ft_sens].measure_direction == iDynTree::FTSensorData::CHILD_TO_PARENT );
//             new_sens.setAppliedWrenchLink(child_link);
//         }
// 
        sensorsTree.addSensor(*newSensor);
    }

    return sensorsTree;
}

}

