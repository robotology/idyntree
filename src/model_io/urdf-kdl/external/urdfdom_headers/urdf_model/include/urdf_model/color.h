/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
/* Author: Josh Faust */

#ifndef URDF_INTERFACE_COLOR_H
#define URDF_INTERFACE_COLOR_H

//For using the M_PI macro in visual studio it
//is necessary to define _USE_MATH_DEFINES
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif


#include <string>
#include <vector>
#include <math.h>
#include "boost_replacements.h"

namespace urdf
{

class Color
{
public:
  Color() {this->clear();};
  float r;
  float g;
  float b;
  float a;

  void clear()
  {
    r = g = b = 0.0f;
    a = 1.0f;
  }
  bool init(const std::string &vector_str)
  {
    this->clear();
    std::vector<std::string> pieces;
    std::vector<double> rgba;
    splitString(vector_str, pieces);
    for (unsigned int i = 0; i < pieces.size(); ++i)
    {
      if (!pieces[i].empty())
      {
        double newDouble;
        if( stringToDouble(pieces[i],newDouble) )
        {
          rgba.push_back(newDouble);
        }
        else
        {
          return false;
        }
      }
    }

    if (rgba.size() != 4)
    {
      return false;
    }

    this->r = (float)rgba[0];
    this->g = (float)rgba[1];
    this->b = (float)rgba[2];
    this->a = (float)rgba[3];

    return true;
  };
};

}

#endif

