/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "iDynTree/ModelIO/URDFGenericSensorsImport.h"
#include "iDynTree/ModelIO/ModelLoader.h"

namespace iDynTree
{

bool sensorsFromURDF(const std::string & urdf_filename,
                     iDynTree::SensorsList & output)
{
    ModelLoader loader;
    bool ok = loader.loadModelFromFile(urdf_filename);
    output = loader.sensors();
    return ok;
}

bool sensorsFromURDF(const std::string & urdf_filename,
                     const Model & model,
                     iDynTree::SensorsList & output)
{
    ModelLoader loader;
    bool ok = loader.loadModelFromFile(urdf_filename);
    output = loader.sensors();

    if (!output.isConsistent(model))
    {
        reportError("", "sensorsFromURDF", "Specified Model is not consistent with the sensors extracted from URDF file."
                        " Please use iDynTree::ModelLoader to parse both model and sensors at the same time.");
        return false;
    }

    return ok;
}

bool sensorsFromURDFString(const std::string & urdf_string,
                           iDynTree::SensorsList & output)
{
    ModelLoader loader;
    bool ok = loader.loadModelFromString(urdf_string);
    output = loader.sensors();
    return ok;
}

bool sensorsFromURDFString(const std::string & urdf_string,
                           const Model & model,
                           iDynTree::SensorsList & output)
{
    ModelLoader loader;
    bool ok = loader.loadModelFromString(urdf_string);
    output = loader.sensors();

    if (!output.isConsistent(model))
    {
        reportError("", "sensorsFromURDFString", "Specified Model is not consistent with the sensors extracted from URDF string."
                        " Please use iDynTree::ModelLoader to parse both model and sensors at the same time.");
        return false;
    }

    return ok;
}

}

