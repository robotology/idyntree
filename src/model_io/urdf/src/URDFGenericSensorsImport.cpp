/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include <iDynTree/ModelIO/URDFGenericSensorsImport.h>

#include <iDynTree/ModelIO/urdf_generic_sensor_import.hpp>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/URDFModelImport.h>

#include <iostream>

namespace iDynTree
{

bool genericSensorsListFromURDF(const std::string & urdf_filename,
                                iDynTree::SensorsList & output)
{
    iDynTree::Model model;
    bool ok = modelFromURDF(urdf_filename,model);

    if( !ok )
    {
        std::cerr << "[ERROR] iDynTree::sensorsListFromURDF : error in loading urdf "
                  << urdf_filename << std::endl;
        return false;
    }

    ok = genericSensorsListFromURDF(model,urdf_filename,output);

    return ok;
}

bool genericSensorsListFromURDFString(const std::string& urdf_string,
                                     iDynTree::SensorsList& output)
{
    iDynTree::Model model;
    bool ok = modelFromURDFString(urdf_string,model);

    if( !ok )
    {
        std::cerr << "[ERROR] iDynTree::sensorsListFromURDFString : error in loading urdf  string "
                  << urdf_string << std::endl;
        return false;
    }


    ok = genericSensorsListFromURDFString(model,urdf_string,output);

    return ok;
}

}

