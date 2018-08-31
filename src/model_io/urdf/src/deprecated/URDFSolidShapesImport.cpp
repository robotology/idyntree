/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "iDynTree/ModelIO/URDFSolidShapesImport.h"
#include "iDynTree/ModelIO/ModelLoader.h"


namespace iDynTree
{

bool solidShapesFromURDF(const std::string & urdf_filename,
                         const Model & model,
                         const std::string urdfGeometryType,
                               ModelSolidShapes & output)
{
    if (urdfGeometryType != "visual" && urdfGeometryType != "collision")
    {
        std::cerr << "[ERROR] unknown urdfGeometryType " << urdfGeometryType << std::endl;
        return false;
    }

    ModelLoader loader;
    bool ok = loader.loadModelFromFile(urdf_filename);

    if (ok && urdfGeometryType == "visual")
    {
        output = loader.model().visualSolidShapes();
    }

    if (ok && urdfGeometryType == "collision")
    {
        output = loader.model().collisionSolidShapes();
    }

    return ok;
}

bool solidShapesFromURDFString(const std::string & urdf_string,
                               const std::string & urdf_filename,
                               const Model & model,
                               const std::string urdfGeometryType,
                                     iDynTree::ModelSolidShapes & output)
{
    if (urdfGeometryType != "visual" && urdfGeometryType != "collision")
    {
        std::cerr << "[ERROR] unknown urdfGeometryType " << urdfGeometryType << std::endl;
        return false;
    }

    ModelLoader loader;
    ModelParserOptions options;
    options.originalFilename = urdf_filename;
    loader.setParsingOptions(options);
    bool ok = loader.loadModelFromString(urdf_string);

    if (ok && urdfGeometryType == "visual")
    {
        output = loader.model().visualSolidShapes();
    }

    if (ok && urdfGeometryType == "collision")
    {
        output = loader.model().collisionSolidShapes();
    }

    return ok;
}

}

