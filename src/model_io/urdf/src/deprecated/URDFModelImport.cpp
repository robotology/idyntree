/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "iDynTree/ModelIO/URDFModelImport.h"

#include "iDynTree/ModelIO/ModelLoader.h"

#include <iDynTree/Model/Model.h>

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <set>

namespace iDynTree
{

inline ModelParserOptions convertOptions(const URDFParserOptions options)
{
    ModelParserOptions retOptions;
    retOptions.originalFilename = options.originalFilename;
    retOptions.addSensorFramesAsAdditionalFrames = options.addSensorFramesAsAdditionalFrames;
    return retOptions;
}

bool modelFromURDF(const std::string & urdf_filename,
                   iDynTree::Model & output,
                   const URDFParserOptions options)
{
    ModelLoader loader;
    loader.setParsingOptions(convertOptions(options));
    bool ok = loader.loadModelFromFile(urdf_filename);
    output = loader.model();
    return ok;
}

bool modelFromURDFString(const std::string & urdf_string,
                         iDynTree::Model & output,
                         const URDFParserOptions options)
{
    ModelLoader loader;
    loader.setParsingOptions(convertOptions(options));
    bool ok = loader.loadModelFromString(urdf_string);
    output = loader.model();
    return ok;
}

}

