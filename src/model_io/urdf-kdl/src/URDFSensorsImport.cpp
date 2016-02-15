/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <iDynTree/ModelIO/URDFSensorsImport.h>

#include <iDynTree/ModelIO/impl/urdf_import.hpp>
#include <iDynTree/ModelIO/impl/urdf_sensor_import.hpp>

#include <iostream>

namespace iDynTree
{

bool sensorsListFromURDF(const std::string & urdf_filename,
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

    KDL::CoDyCo::UndirectedTree undirected_tree = KDL::CoDyCo::UndirectedTree(tree);

    output = sensorsListFromURDF(undirected_tree,urdf_filename);

    return ok;
}

bool sensorsListFromURDFString(const std::string& urdf_string,
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

    KDL::CoDyCo::UndirectedTree undirected_tree = KDL::CoDyCo::UndirectedTree(tree);

    output = sensorsListFromURDFString(undirected_tree,urdf_string);

    return ok;
}

}

