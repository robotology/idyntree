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


#include <urdf_model_state/model_state.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <tinyxml.h>
#include "urdf_parser/outputdecl.h"
#include <urdf_model/boost_replacements.h>

namespace urdf{

bool parseModelState(ModelState &ms, TiXmlElement* config)
{
  ms.clear();

  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    logError("No name given for the model_state.");
    return false;
  }
  ms.name = std::string(name_char);

  const char *time_stamp_char = config->Attribute("time_stamp");
  if (time_stamp_char)
  {
    double newDouble;
    if( stringToDouble(time_stamp_char,newDouble) )
    {
      double sec = newDouble;
      ms.time_stamp.set(sec);
    }
    else {
      logError("Parsing time stamp [%s] failed.", time_stamp_char);
      return false;
    }
  }

  TiXmlElement *joint_state_elem = config->FirstChildElement("joint_state");
  if (joint_state_elem)
  {
    boost::shared_ptr<JointState> joint_state;
    joint_state.reset(new JointState());

    const char *joint_char = joint_state_elem->Attribute("joint");
    if (joint_char)
      joint_state->joint = std::string(joint_char);
    else
    {
      logError("No joint name given for the model_state.");
      return false;
    }

    // parse position
    const char *position_char = joint_state_elem->Attribute("position");
    if (position_char)
    {

      std::vector<std::string> pieces;
      splitString( position_char, pieces);
      for (unsigned int i = 0; i < pieces.size(); ++i){
        double newDouble;
        if (pieces[i] != ""){
          if( stringToDouble(pieces[i],newDouble) )
          {
            joint_state->position.push_back(newDouble);
          } else {
            throw ParseError("position element ("+ pieces[i] +") is not a valid float");
          }
        }
      }
    }

    // parse velocity
    const char *velocity_char = joint_state_elem->Attribute("velocity");
    if (velocity_char)
    {

      std::vector<std::string> pieces;
      splitString( position_char, pieces);

      for (unsigned int i = 0; i < pieces.size(); ++i){

        if (pieces[i] != ""){
          double newDouble;
          if( stringToDouble(time_stamp_char,newDouble) )
          {
            joint_state->velocity.push_back(newDouble);
          } else {
            throw ParseError("velocity element ("+ pieces[i] +") is not a valid float");
          }
        }
      }
    }

    // parse effort
    const char *effort_char = joint_state_elem->Attribute("effort");
    if (effort_char)
    {

      std::vector<std::string> pieces;
      splitString( position_char, pieces);

      for (unsigned int i = 0; i < pieces.size(); ++i){
        if (pieces[i] != ""){
          double newDouble;
          if( stringToDouble(time_stamp_char,newDouble) )
          {
            joint_state->effort.push_back(newDouble);
          } else {
            throw ParseError("effort element ("+ pieces[i] +") is not a valid float");
          }
        }
      }
    }

    // add to vector
    ms.joint_states.push_back(joint_state);
  }
};



}


