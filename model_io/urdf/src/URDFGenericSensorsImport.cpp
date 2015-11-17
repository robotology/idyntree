/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <iDynTree/ModelIO/URDFGenericSensorsImport.h>

#include <iDynTree/ModelIO/impl/urdf_import.hpp>
#include <iDynTree/ModelIO/impl/urdf_generic_sensor_import.hpp>

#include <iostream>

namespace iDynTree
{

bool genericSensorsListFromURDF(const std::string & urdf_filename,
                         iDynTree::SensorsList & output)
{
    // we get first the KDL::CoDyCo::UndirectedTree object,
    // and then we used it to parse also sensors informations.
    KDL::Tree tree;
    bool ok = treeFromUrdfFile(urdf_filename,tree);

    if( !ok )
    {
        std::cerr << "iDynTree::sensorsListFromURDF : error in loading urdf "
                  << urdf_filename << std::endl;
        return false;
    }
   
    KDL::CoDyCo::UndirectedTree undirectedTree(tree);
    output = genericSensorsListFromURDF(undirectedTree,urdf_filename);

    return ok;
}

bool genericSensorsListFromURDFString(const std::string& urdf_string,
                               iDynTree::SensorsList& output)
{
    // we get first the KDL::CoDyCo::UndirectedTree object,
    // and then we used it to parse also sensors informations.
    KDL::Tree tree;
    bool ok = treeFromUrdfString(urdf_string,tree);

    if( !ok )
    {
        std::cerr << "iDynTree::sensorsListFromURDFString : error in loading urdf  string "
                  << urdf_string << std::endl;
        return false;
    }
    
#ifdef DEBUG
    std::cout<<"URDFGenericSensorsImport: tree loaded, now trying to call the parser function\n";
#endif //DEBUG
    
    KDL::CoDyCo::UndirectedTree undirectedTree = KDL::CoDyCo::UndirectedTree(tree);
    output = genericSensorsListFromURDFString(undirectedTree,urdf_string);

    return ok;
}

}

