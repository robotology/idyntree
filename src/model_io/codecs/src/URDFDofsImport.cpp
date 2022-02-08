/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "iDynTree/ModelIO/URDFDofsImport.h"

#include "iDynTree/ModelIO/ModelLoader.h"

#include <iDynTree/Model/Model.h>

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <set>

namespace iDynTree
{

bool dofsListFromURDF(const std::string & urdf_filename,
                      std::vector<std::string>& dofs)
{
    std::ifstream ifs(urdf_filename.c_str());

    if( !ifs.is_open() )
    {
        std::cerr << "[ERROR] iDynTree::dofsFromURDF : error opening file "
                  << urdf_filename << std::endl;
        return false;
    }

    std::string xml_string( (std::istreambuf_iterator<char>(ifs) ),
                            (std::istreambuf_iterator<char>()    ) );

    return dofsListFromURDFString(xml_string,dofs);
}

bool dofsListFromURDFString(const std::string & urdf_string,
                            std::vector<std::string>& dofs)
{
    // clear the vector
    dofs.resize(0);

    // Load a iDynTree::Model and dump the dofs name in the vector

    ModelLoader loader;
    bool ok = loader.loadModelFromString(urdf_string);
    iDynTree::Model model = loader.model();

    for(size_t jnt = 0; jnt < model.getNrOfJoints(); jnt++)
    {
        if( model.getJoint(jnt)->getNrOfDOFs() > 0 )
        {
            dofs.push_back(model.getJointName(jnt));
        }
    }

    return ok;
}

}

