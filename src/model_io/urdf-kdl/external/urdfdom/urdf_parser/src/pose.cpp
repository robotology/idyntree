/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
/* Author: Wim Meeussen, John Hsu */


#include <urdf_model/pose.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include "urdf_parser/outputdecl.h"
#include <tinyxml.h>
#include <urdf_parser/urdf_parser.h>

namespace urdf_export_helpers {

std::string values2str(unsigned int count, const double *values, double (*conv)(double))
{
    std::stringstream ss;
    for (unsigned int i = 0 ; i < count ; i++)
    {
        if (i > 0)
            ss << " ";
        ss << (conv ? conv(values[i]) : values[i]);
    }
    return ss.str();
}
std::string values2str(urdf::Vector3 vec)
{
    double xyz[3];
    xyz[0] = vec.x;
    xyz[1] = vec.y;
    xyz[2] = vec.z;
    return values2str(3, xyz);
}
std::string values2str(urdf::Rotation rot)
{
    double rpy[3];
    rot.getRPY(rpy[0], rpy[1], rpy[2]);
    return values2str(3, rpy);
}
std::string values2str(urdf::Color c)
{
    double rgba[4];
    rgba[0] = c.r;
    rgba[1] = c.g;
    rgba[2] = c.b;
    rgba[3] = c.a;
    return values2str(4, rgba);
}
std::string values2str(double d)
{
    return values2str(1, &d);
}
}

namespace urdf{

bool parsePose(Pose &pose, TiXmlElement* xml)
{
  pose.clear();
  if (xml)
  {
    const char* xyz_str = xml->Attribute("xyz");
    if (xyz_str != NULL)
    {
      try {
        pose.position.init(xyz_str);
      }
      catch (ParseError &e) {
        logError(e.what());
        return false;
      }
    }

    const char* rpy_str = xml->Attribute("rpy");
    if (rpy_str != NULL)
    {
      try {
        pose.rotation.init(rpy_str);
      }
      catch (ParseError &e) {
        logError(e.what());
        return false;
      }
    }
  }
  return true;
}

bool exportPose(Pose &pose, TiXmlElement* xml)
{
  TiXmlElement *origin = new TiXmlElement("origin");
  std::string pose_xyz_str = urdf_export_helpers::values2str(pose.position);
  std::string pose_rpy_str = urdf_export_helpers::values2str(pose.rotation);
  origin->SetAttribute("xyz", pose_xyz_str);
  origin->SetAttribute("rpy", pose_rpy_str);
  xml->LinkEndChild(origin);
  return true;
}

}


