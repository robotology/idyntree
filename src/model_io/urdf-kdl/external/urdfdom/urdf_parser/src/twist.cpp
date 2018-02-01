/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
/* Author: John Hsu */


#include <urdf_model/twist.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <tinyxml.h>
#include "urdf_parser/outputdecl.h"

namespace urdf{

bool parseTwist(Twist &twist, TiXmlElement* xml)
{
  twist.clear();
  if (xml)
  {
    const char* linear_char = xml->Attribute("linear");
    if (linear_char != NULL)
    {
      try {
        twist.linear.init(linear_char);
      }
      catch (ParseError &e) {
        twist.linear.clear();
        logError("Malformed linear string [%s]: %s", linear_char, e.what());
        return false;
      }
    }

    const char* angular_char = xml->Attribute("angular");
    if (angular_char != NULL)
    {
      try {
        twist.angular.init(angular_char);
      }
      catch (ParseError &e) {
        twist.angular.clear();
        logError("Malformed angular [%s]: %s", angular_char, e.what());
        return false;
      }
    }
  }
  return true;
}

}



