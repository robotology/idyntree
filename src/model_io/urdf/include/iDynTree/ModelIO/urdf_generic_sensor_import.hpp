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

#ifndef URDF_GENERIC_SENSOR_IMPORT_HPP
#define URDF_GENERIC_SENSOR_IMPORT_HPP

#include <string>
#include <vector>
#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Core/Transform.h>

namespace iDynTree
{
    class Model;
    class Transform;


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



bool genericSensorsFromUrdfFile(const std::string& file, std::vector<GenericSensorData> & generic_sensors);
bool genericSensorsFromUrdfString(const std::string& urdf_xml, std::vector<GenericSensorData> & generic_sensors);

bool  genericSensorsListFromURDF(iDynTree::Model & undirected_tree,
                                                std::string urdf_filename,
                                                 iDynTree::SensorsList & sensors);

bool genericSensorsListFromURDFString(iDynTree::Model & undirected_tree,
                                                       std::string urdf_string,
                                                       iDynTree::SensorsList & sensors);
}

#endif
