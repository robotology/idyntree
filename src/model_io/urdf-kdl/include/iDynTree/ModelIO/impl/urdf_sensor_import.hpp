/*
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
/* Author: Silvio Traversaro */

#ifndef URDF_SENSOR_IMPORT_H
#define URDF_SENSOR_IMPORT_H

#include <string>
#include <vector>

#include <kdl/frames.hpp>

#include <kdl_codyco/undirectedtree.hpp>


class TiXmlDocument;

namespace iDynTree
{
class SensorsList;

struct FTSensorData
{
    std::string reference_joint;
    std::string sensor_name;
    enum { PARENT_LINK_FRAME ,
           CHILD_LINK_FRAME  ,
           SENSOR_FRAME } frame;
    KDL::Frame sensor_pose;
    enum { PARENT_TO_CHILD,
           CHILD_TO_PARENT }
          measure_direction;
};

std::vector<std::string> &split(const std::string &s, std::vector<std::string> &elems) ;

bool ftSensorsFromUrdfFile(const std::string& file, std::vector<FTSensorData> & ft_sensors);
bool ftSensorsFromUrdfString(const std::string& urdf_xml, std::vector<FTSensorData> & ft_sensors);

iDynTree::SensorsList sensorsListFromURDF(KDL::CoDyCo::UndirectedTree & undirected_tree,
                                          std::string urdf_filename);
iDynTree::SensorsList sensorsListFromURDFString(KDL::CoDyCo::UndirectedTree & undirected_tree,
                                                std::string urdf_string);
iDynTree::SensorsList sensorsListFromFtSensors(KDL::CoDyCo::UndirectedTree & undirected_tree,
                                               std::vector<FTSensorData>& ft_sensors);
}

#endif
