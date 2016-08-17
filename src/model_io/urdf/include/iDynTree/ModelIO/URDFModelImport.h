/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef IDYNTREE_URDF_MODEL_IMPORT_H
#define IDYNTREE_URDF_MODEL_IMPORT_H

#include <string>

namespace iDynTree

{

class Model;

/**
 * \ingroup iDynTreeModelIO
 *
 * Options for the iDynTree URDF parser.
 */
struct URDFParserOptions
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
 */
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
 */
bool modelFromURDFString(const std::string & urdf_string,
                         iDynTree::Model & output,
                         const URDFParserOptions options=URDFParserOptions());


}

#endif