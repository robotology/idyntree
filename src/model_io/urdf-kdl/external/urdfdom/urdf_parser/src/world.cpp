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


#include <urdf_world/world.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <tinyxml.h>
#include "urdf_parser/outputdecl.h"

namespace urdf{

bool parseWorld(World &world, TiXmlElement* config)
{

  // to be implemented

  return true;
}

bool exportWorld(World &world, TiXmlElement* xml)
{
  TiXmlElement * world_xml = new TiXmlElement("world");
  world_xml->SetAttribute("name", world.name);

  // to be implemented
  // exportModels(*world.models, world_xml);

  xml->LinkEndChild(world_xml);

  return true;
}

}
