// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "iDynTree/URDFDofsImport.h"

#include "iDynTree/ModelLoader.h"

#include <iDynTree/Model.h>

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

