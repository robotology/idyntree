/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <iDynTree/ModelIO/URDFDofsImport.h>

#include <iDynTree/ModelIO/URDFModelImport.h>

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
    iDynTree::Model model;

    bool ok = modelFromURDFString(urdf_string,model);

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

