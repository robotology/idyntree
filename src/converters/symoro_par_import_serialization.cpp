/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2013, Istituto Italiano di Tecnologia
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

#include "kdl_format_io/symoro_par_import_serialization.hpp"

#include "../expression_parser/parser.h"
#include <string>
#include <boost/iterator/iterator_concepts.hpp>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <utility>

using namespace KDL;
using namespace std;

namespace kdl_format_io {
    
bool treeSerializationFromSymoroParFile(const string& parfile_name, KDL::CoDyCo::TreeSerialization& serialization, const bool consider_first_link_inertia)
{
    ifstream ifs(parfile_name.c_str());
    std::string xml_string( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );

    return treeSerializationFromSymoroParString(xml_string,serialization,consider_first_link_inertia);
}

bool treeSerializationFromSymoroParString(const string& parfile_name, KDL::CoDyCo::TreeSerialization& serialization, const bool consider_first_link_inertia)
{
    symoro_par_model par_model;
    if( !parModelFromString(parfile_name,par_model) ) return false;
    
    return treeSerializationFromParModel(par_model,serialization,consider_first_link_inertia);
}


std::string int2string_serialization(const int in)
{
    std::stringstream ss;
    ss << in;
    return ss.str();
}




bool treeSerializationFromParModelTree(const symoro_par_model& par_model, KDL::CoDyCo::TreeSerialization& serialization, const bool consider_first_link_inertia)
{
    /** \todo move this names in general header */
    const std::string base_name = "Link0";
    
    //As the SYMORO .par doesn't support names for link and joints, the links will be named Link1, Link2, and joints Joint1, Joint2
    const std::string link_common_name = "Link";
    const std::string joint_common_name = "Joint";
    
    std::vector<std::string> link_names(par_model.NL+1);
    std::vector<std::string> joint_names(par_model.NJ+1);
    
    //Return value
    serialization.setNrOfLinks(par_model.NL+1);
    serialization.setNrOfJunctions(par_model.NL);
    serialization.setNrOfDOFs(par_model.NJ);
    
    int link_cnt = 0;
    int dof_cnt = 0;
    int fixed_junction_cnt = 0;
        
    /** \todo fix inconsistencies */
    //serialization.links[link_cnt] = base_name;
    serialization.setLinkNameID(base_name,link_cnt);
    link_cnt++;
    
    for(int l=0; l < par_model.NL; l++ ) {
        std::string link_name = link_common_name + int2string_serialization(l+1);
        std::string joint_name = joint_common_name + int2string_serialization(l+1);
        //serialization.links[link_cnt] = link_name;
        serialization.setLinkNameID(link_name,link_cnt);
        link_cnt++;
                                  
        switch( par_model.Sigma[l] ) {
            case 0:
            case 1:
                //Moving joint
                //serialization.dofs[dof_cnt] = joint_name;
                //serialization.junctions[dof_cnt] = joint_name;
                serialization.setDOFNameID(joint_name,dof_cnt);
                serialization.setJunctionNameID(joint_name,dof_cnt);
                dof_cnt++;
            break;
            case 2: 
                //Fixed joint
                //serialization.junctions[serialization.getNrOfDOFs()+fixed_junction_cnt] = joint_name;
                serialization.setJunctionNameID(joint_name,serialization.getNrOfDOFs()+fixed_junction_cnt);
                fixed_junction_cnt++;
            break;
            default:
            std::cerr << "Error: Sigma value not expected"<< std::endl; return false;
            break;
        }
        
       
    }
    return true;
    
}

bool treeSerializationFromParModelChain(const symoro_par_model& par_model, KDL::CoDyCo::TreeSerialization& serialization, const bool consider_first_link_inertia)
{
    return false;   
}

bool treeSerializationFromParModel(const symoro_par_model& par_model, KDL::CoDyCo::TreeSerialization& serialization, const bool consider_first_link_inertia)
{
    if( par_model.Type != 1 && par_model.Type != 0 ) { 
        std::cerr << "Error: currently are only supported SYMORO+ .par files of Type Tree (1) and Simple Chain (0)" << std::endl;
        return false;
    }
    
    if( !par_model.isConsistent() ) {
        std::cerr << "Error: the SYMORO par model is not consistent" << std::endl;
        return false;
    }
    
    if( par_model.Type == 1 ) return treeSerializationFromParModelTree(par_model,serialization,consider_first_link_inertia);
    if( par_model.Type == 0 ) return treeSerializationFromParModelChain(par_model,serialization,consider_first_link_inertia);
   
    std::cerr << "Error: currently are only supported SYMORO+ .par files of Type Tree (1) and Simple Chain (0)" << std::endl;
    return false;
}


}
