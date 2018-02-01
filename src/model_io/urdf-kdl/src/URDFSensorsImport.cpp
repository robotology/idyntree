/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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

