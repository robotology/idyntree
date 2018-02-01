/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

/* Author: Silvio Traversaro */

#include <iDynTree/ModelIO/symoro_par_import_serialization.hpp>

#include <expression_parser/parser.h>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <utility>
#include <kdl/tree.hpp>
#include <kdl_codyco/treeserialization.hpp>

using namespace KDL;
using namespace std;

namespace iDynTree {

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




bool treeSerializationFromParModelTree(const symoro_par_model& par_model, KDL::CoDyCo::TreeSerialization& serialization, const bool /*consider_first_link_inertia*/)
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

    for(size_t l=0; l < par_model.NL; l++ ) {
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

bool treeSerializationFromParModelChain(const symoro_par_model& /*par_model*/, KDL::CoDyCo::TreeSerialization& /*serialization*/, const bool /*consider_first_link_inertia*/)
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
