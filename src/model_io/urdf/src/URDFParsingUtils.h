/**
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef IDYNTREE_URDF_PARSING_UTILS_H
#define IDYNTREE_URDF_PARSING_UTILS_H

#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorFixSize.h>

#include <tinyxml.h>

namespace iDynTree
{


bool inline stringToDouble(const std::string & inStr, double & outDouble)
{
    outDouble = std::atof(inStr.c_str());
    return true;
}

bool inline stringToInt(const std::string & inStr, int & outInt)
{
    outInt = std::atoi(inStr.c_str());
    return true;
}

bool inline stringToUnsignedInt(const std::string & inStr, unsigned int & outInt)
{
    outInt = (unsigned int)std::atoi(inStr.c_str());
    return true;
}

std::string inline intToString(const int inInt)
{
    std::stringstream ss;
    ss << inInt;
    return ss.str();
}

/**
 * Split string along spaces
 */
bool inline splitString(const std::string & inStr, std::vector<std::string> & pieces)
{
    std::istringstream iss(inStr);

    pieces.resize(0);

    do
    {
        std::string sub;
        iss >> sub;
        if( sub != "" )
        {
            pieces.push_back(sub);
        }
    } while (iss);


    return true;
}

/**
 *
 */
bool inline vector3FromString(const std::string & vector_str, Vector3 & out)
{
    std::vector<std::string> pieces;
    std::vector<double> xyz;
    splitString(vector_str,pieces);
    for (unsigned int i = 0; i < pieces.size(); ++i)
    {
      if (pieces[i] != ""){
       double newDouble;
       if( stringToDouble(pieces[i],newDouble) )
       {
          xyz.push_back(newDouble);
       }
       else
       {
           std::string errStr = "Unable to parse component [" + pieces[i] + "] to a double (while parsing a vector value)";
           reportError("","vector3FromString",errStr.c_str());
           return false;
       }
      }
    }

    if (xyz.size() != 3)
    {
        std::string errStr = "Parser found " + intToString(xyz.size())  + " elements but 3 expected while parsing vector [" + vector_str + "]";
        reportError("","vector3FromString",errStr.c_str());
        return false;
    }

    out(0) = xyz[0];
    out(1) = xyz[1];
    out(2) = xyz[2];

    return true;
}


bool inline transformFromURDFXML(TiXmlElement* xml,
                                 Transform & transform)
{
    // Default transform is identify
    transform = Transform::Identity();

    if (xml)
    {
        const char* xyz_str = xml->Attribute("xyz");
        if (xyz_str != NULL)
        {
            Position xyz;
            if( vector3FromString(xyz_str,xyz) )
            {
                transform.setPosition(xyz);
            }
            else
            {
                return false;
            }
        }

        const char* rpy_str = xml->Attribute("rpy");
        if (rpy_str != NULL)
        {
            Vector3 rpy;
            if( vector3FromString(rpy_str,rpy) )
            {
                transform.setRotation(Rotation::RPY(rpy(0),rpy(1),rpy(2)));
            }
            else
            {
                return false;
            }
        }
    }
    return true;
}

bool inline vector4FromString(const std::string & vector_str, Vector4 & out)
{
    std::vector<std::string> pieces;
    std::vector<double> rgba;
    splitString(vector_str,pieces);
    for (unsigned int i = 0; i < pieces.size(); ++i)
    {
      if (pieces[i] != ""){
       double newDouble;
       if( stringToDouble(pieces[i],newDouble) )
       {
          rgba.push_back(newDouble);
       }
       else
       {
           std::string errStr = "Unable to parse component [" + pieces[i] + "] to a double (while parsing a vector value)";
           reportError("","vector4FromString",errStr.c_str());
           return false;
       }
      }
    }

    if (rgba.size() != 4)
    {
        std::string errStr = "Parser found " + intToString(rgba.size())  + " elements but 4 expected while parsing vector [" + vector_str + "]";
        reportError("","vector4FromString",errStr.c_str());
        return false;
    }

    out(0) = rgba[0];
    out(1) = rgba[1];
    out(2) = rgba[2];
    out(3) = rgba[3];

    return true;
}

}

#endif
