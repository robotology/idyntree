/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
/* Author: Wim Meeussen */

#ifndef URDF_PARSER_URDF_PARSER_H
#define URDF_PARSER_URDF_PARSER_H

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <string>
#include <map>
#include <tinyxml.h>
#include <urdf_model/model.h>
#include <urdf_model/color.h>

#include "exportdecl.h"

namespace urdf_export_helpers {

 std::string values2str(unsigned int count, const double *values, double (*conv)(double) = NULL);
 std::string values2str(urdf::Vector3 vec);
 std::string values2str(urdf::Rotation rot);
 std::string values2str(urdf::Color c);
 std::string values2str(double d);

}

namespace urdf{

   ModelInterfacePtr parseURDF(const std::string &xml_string);
   ModelInterfacePtr parseURDFFile(const std::string &path);
   TiXmlDocument*  exportURDF(ModelInterfacePtr &model);
   TiXmlDocument*  exportURDF(const ModelInterface &model);
   bool parsePose(Pose&, TiXmlElement*);
}

#endif
