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

/* Author: Silvio Traversaro */

#include <iDynTree/ModelIO/impl/urdf_sensor_import.hpp>

#include <iDynTree/Core/Transform.h>
#include <kdl_codyco/undirectedtree.hpp>

#include <kdl_codyco/KDLConversions.h>

#include <iDynTree/Sensors/Sensors.hpp>
#include <iDynTree/Sensors/SixAxisFTSensor.hpp>


#include <fstream>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <tinyxml.h>

using namespace std;
using namespace KDL;

namespace iDynTree{


bool ftSensorsFromUrdfFile(const std::string& file, std::vector<FTSensorData> & ft_sensors)
{
    ifstream ifs(file.c_str());
    std::string xml_string( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );

    return ftSensorsFromUrdfString(xml_string,ft_sensors);
}

std::vector<std::string> &split(const std::string &s, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (ss >> item) {
        elems.push_back(item);
    }
    return elems;
}


bool ftSensorsFromUrdfString(const std::string& urdf_xml, std::vector<FTSensorData> & ft_sensors)
{
    ft_sensors.resize(0);

    /* parse sdf force_torque extension to urdf */
    TiXmlDocument urdfXml;
    urdfXml.Parse(urdf_xml.c_str());

    TiXmlElement* robotXml = urdfXml.FirstChildElement("robot");

    // Get all SDF extension elements and search for the ft sensors
    for (TiXmlElement* sdfXml = robotXml->FirstChildElement("gazebo");
         sdfXml; sdfXml = sdfXml->NextSiblingElement("gazebo"))
    {
        const char* ref = sdfXml->Attribute("reference");
        if (ref)
        {
            for (TiXmlElement* sensorXml = sdfXml->FirstChildElement("sensor");
                sensorXml; sensorXml = sensorXml->NextSiblingElement("sensor"))
            {
                  const char* sensor_type = sensorXml->Attribute("type");
                  if( sensor_type != NULL && std::string(sensor_type) == "force_torque")
                  {
                      //Add the found ft sensor to the list
                      FTSensorData new_ft;
                      std::string refStr = std::string(ref);
                      new_ft.reference_joint = refStr;
                      new_ft.sensor_name = sensorXml->Attribute("name");
                      // Default value, check sdf documentation
                      new_ft.frame = iDynTree::FTSensorData::CHILD_LINK_FRAME;
                      new_ft.measure_direction = iDynTree::FTSensorData::CHILD_TO_PARENT;
                      new_ft.sensor_pose = KDL::Frame::Identity();
                      TiXmlElement* force_torque_tags = sensorXml->FirstChildElement("force_torque");
                      if( force_torque_tags )
                      {
                          TiXmlElement* frame_tag = sensorXml->FirstChildElement("frame");
                          if( frame_tag )
                          {
                              std::string frame_text = frame_tag->GetText();
                              if( frame_text == "child" )
                              {
                                  new_ft.frame = iDynTree::FTSensorData::CHILD_LINK_FRAME;
                              }
                              else if ( frame_text == "parent" )
                              {
                                  new_ft.frame = iDynTree::FTSensorData::PARENT_LINK_FRAME;
                              }
                              else if ( frame_text == "sensor" )
                              {
                                  new_ft.frame = iDynTree::FTSensorData::SENSOR_FRAME;
                              }
                              else
                              {
                                  return false;
                              }

                          }
                          TiXmlElement* measure_direction_tag = sensorXml->FirstChildElement("measure_direction");
                          if( measure_direction_tag )
                          {
                              std::string measure_direction_text = measure_direction_tag->GetText();
                              if (measure_direction_text == "child_to_parent")
                              {
                                  new_ft.measure_direction = iDynTree::FTSensorData::CHILD_TO_PARENT;
                              }
                              else if (measure_direction_text == "parent_to_child" )
                              {
                                  new_ft.measure_direction = iDynTree::FTSensorData::PARENT_TO_CHILD;
                              }
                              else
                              {
                                  return false;
                              }
                          }
                      }
                      TiXmlElement* pose_tag = sensorXml->FirstChildElement("pose");
                      if( pose_tag )
                      {
                          std::string  pose_text = pose_tag->GetText();
                          std::vector<std::string> pose_elems;
                          split(pose_text,pose_elems);
                          if( pose_elems.size() != 6 )
                          {
                              return false;
                          }
                          double roll  = atof(pose_elems[3].c_str());
                          double pitch = atof(pose_elems[4].c_str());
                          double yaw   = atof(pose_elems[5].c_str());
                          new_ft.sensor_pose.M = KDL::Rotation::RPY(roll,pitch,yaw);
                          double x  = atof(pose_elems[0].c_str());
                          double y = atof(pose_elems[1].c_str());
                          double z   = atof(pose_elems[2].c_str());
                          new_ft.sensor_pose.p = KDL::Vector(x,y,z);
                      }

                      ft_sensors.push_back(new_ft);

                  }
            }
        }
    }

    return true;
}

iDynTree::SensorsList sensorsListFromURDF(KDL::CoDyCo::UndirectedTree & undirected_tree,
                                          std::string urdf_filename)
{
    ifstream ifs(urdf_filename.c_str());
    std::string xml_string( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );

    return sensorsListFromURDFString(undirected_tree,xml_string);
}

iDynTree::SensorsList sensorsListFromURDFString(KDL::CoDyCo::UndirectedTree & undirected_tree,
                                                std::string urdf_string)
{
    iDynTree::SensorsList sensors_tree;

    std::vector<iDynTree::FTSensorData> ft_sensors;
    bool ok = iDynTree::ftSensorsFromUrdfString(urdf_string,ft_sensors);

    if( !ok )
    {
        std::cerr << "Error in loading ft sensors information from URDF file" << std::endl;
    }

    for(int ft_sens = 0; ft_sens < ft_sensors.size(); ft_sens++ )
    {
        iDynTree::SixAxisForceTorqueSensor new_sens;

        // Convert the information in the FTSensorData format in
        // a series of SixAxisForceTorqueSensor objects, using the
        // serialization provided in the undirected_tree object
        new_sens.setName(ft_sensors[ft_sens].reference_joint);

        new_sens.setParent(ft_sensors[ft_sens].reference_joint);

        KDL::CoDyCo::JunctionMap::const_iterator junct_it
            = undirected_tree.getJunction(ft_sensors[ft_sens].reference_joint);

        new_sens.setParentIndex(junct_it->getJunctionIndex());

        int parent_link = junct_it->getParentLink()->getLinkIndex();
        int child_link = junct_it->getChildLink()->getLinkIndex();
        std::string parent_link_name = junct_it->getParentLink()->getName();
        std::string child_link_name = junct_it->getChildLink()->getName();

        KDL::Frame parent_link_H_child_link = junct_it->pose(0.0,false);
        KDL::Frame child_link_H_sensor = ft_sensors[ft_sens].sensor_pose;

        // For now we assume that the six axis ft sensor is attached to a
        // fixed junction. Hence the first/second link to sensor transforms
        // are fixed are given by the frame option
        if( ft_sensors[ft_sens].frame == iDynTree::FTSensorData::PARENT_LINK_FRAME )
        {
            new_sens.setFirstLinkSensorTransform(parent_link,iDynTree::Transform::Identity());
            new_sens.setSecondLinkSensorTransform(child_link,iDynTree::ToiDynTree(parent_link_H_child_link.Inverse()));
        }
        else if( ft_sensors[ft_sens].frame == iDynTree::FTSensorData::CHILD_LINK_FRAME )
        {
            new_sens.setFirstLinkSensorTransform(parent_link,iDynTree::ToiDynTree(parent_link_H_child_link));
            new_sens.setSecondLinkSensorTransform(child_link,iDynTree::Transform::Identity());
        }
        else
        {
            assert( ft_sensors[ft_sens].frame == iDynTree::FTSensorData::SENSOR_FRAME );
            new_sens.setFirstLinkSensorTransform(parent_link,iDynTree::ToiDynTree(parent_link_H_child_link*child_link_H_sensor));
            new_sens.setSecondLinkSensorTransform(child_link,iDynTree::ToiDynTree(child_link_H_sensor));
        }

        //set names
        new_sens.setFirstLinkName(parent_link_name);
        new_sens.setSecondLinkName(child_link_name);

        if( ft_sensors[ft_sens].measure_direction == iDynTree::FTSensorData::CHILD_TO_PARENT )
        {
            new_sens.setAppliedWrenchLink(parent_link);
        }
        else
        {
            assert( ft_sensors[ft_sens].measure_direction == iDynTree::FTSensorData::CHILD_TO_PARENT );
            new_sens.setAppliedWrenchLink(child_link);
        }

        sensors_tree.addSensor(new_sens);
    }

    return sensors_tree;
}

}

