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

#include "kdl_format_io/urdf_sensor_import.hpp"
#include <fstream>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <tinyxml.h>

using namespace std;
using namespace KDL;

namespace kdl_format_io{


bool ftSensorsFromUrdfFile(const std::string& file, std::vector<FTSensorData> & ft_sensors)
{
    ifstream ifs(file.c_str());
    std::string xml_string( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );

    return ftSensorsFromUrdfFile(xml_string,ft_sensors);
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
                      new_ft.frame = kdl_format_io::FTSensorData::CHILD_LINK_FRAME;
                      new_ft.measure_direction = kdl_format_io::FTSensorData::CHILD_TO_PARENT;
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
                                  new_ft.frame = kdl_format_io::FTSensorData::CHILD_LINK_FRAME;
                              }
                              else if ( frame_text == "parent" )
                              {
                                  new_ft.frame = kdl_format_io::FTSensorData::PARENT_LINK_FRAME;
                              }
                              else if ( frame_text == "sensor" )
                              {
                                  new_ft.frame = kdl_format_io::FTSensorData::SENSOR_FRAME;
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
                                  new_ft.measure_direction = kdl_format_io::FTSensorData::CHILD_TO_PARENT;
                              }
                              else if (measure_direction_text == "parent_to_child" )
                              {
                                  new_ft.measure_direction = kdl_format_io::FTSensorData::PARENT_TO_CHILD;
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
}

}

