/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_URDF_MODEL_IMPORT_H
#define IDYNTREE_URDF_MODEL_IMPORT_H

#ifdef __DEPRECATED
  #warning <iDynTree/ModelIO/URDFModelImport.h> is deprecated. Please use <iDynTree/ModelIO/ModelLoader.h> and the classes contained in it. To disable this warning use -Wno-deprecated.
#endif

#include <string>

#include <iDynTree/Core/Utils.h>

namespace iDynTree

{

class Model;

/**
 * \ingroup iDynTreeModelIO
 *
 * Options for the iDynTree URDF parser.
 */

struct IDYNTREE_DEPRECATED URDFParserOptions
{
    /**
     * If true, add to the model the sensor frames
     * as additional frames with the same name of the sensor.
     * If there is already a link or additional frame with the same
     * name of the sensor, a warning is printed and no frame is added.
     */
    bool addSensorFramesAsAdditionalFrames;

    /**
     * Original filename of the URDF sensor parsed.
     *
     * This attribute is the original filename of the URDF sensor parsed.
     * It is useful when loading a model from a string, if that URDF string
     * has <geometry> tags that point to external meshes. To find the location
     * of this external meshes, we need also the original filename of the URDF file.
     */
    std::string originalFilename;

    /**
     * Constructor, containing default values.
     */
    URDFParserOptions(): addSensorFramesAsAdditionalFrames(true),
                         originalFilename("")
    {
    }
};

/**
 * \ingroup iDynTreeModelIO
 *
 * Load a iDynTree::Model object from a URDF file.
 *
 * Loading the URDF with this function, the  "fake links"
 * present in the URDF file are   removed from the models
 * and added back as frames. See the removeFakeLinks
 * function, that is called inside this function before
 * returning the model.
 *
 * @param[in] options the URDFParserOptions struct of options passed to the parser
 * @return true if all went ok, false otherwise.
 *
 * @deprecated Please use iDynTree::ModelLoader::loadModelFromFile to load a iDynTree::Model from a URDF file.
 */
IDYNTREE_DEPRECATED_WITH_MSG("Please use iDynTree::ModelLoader::loadModelFromFile to load a iDynTree::Model from a URDF file.")
bool modelFromURDF(const std::string & urdf_filename,
                   iDynTree::Model & output,
                   const URDFParserOptions options=URDFParserOptions());

/**
 * \ingroup iDynTreeModelIO
 *
 * Loading the URDF with this function, the  "fake links"
 * present in the URDF file are   removed from the models
 * and added back as frames. See the removeFakeLinks
 * function, that is called inside this function before
 * returning the model. .
 *
 * @param[in] options the URDFParserOptions struct of options passed to the parser
 * @return true if all went ok, false otherwise.
 *
 * @deprecated Please use iDynTree::ModelLoader::loadModelFromString to load a iDynTree::Model from a URDF string.
 */
IDYNTREE_DEPRECATED_WITH_MSG("Please use iDynTree::ModelLoader::loadModelFromString to load a iDynTree::Model from a URDF string.")
bool modelFromURDFString(const std::string & urdf_string,
                         iDynTree::Model & output,
                         const URDFParserOptions options=URDFParserOptions());


}

#endif
