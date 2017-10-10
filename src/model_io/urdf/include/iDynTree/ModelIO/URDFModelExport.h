/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef IDYNTREE_URDF_MODEL_EXPORT_H
#define IDYNTREE_URDF_MODEL_EXPORT_H

#include <string>

namespace iDynTree

{

class Model;

/**
 * \ingroup iDynTreeModelIO
 *
 * Options for the iDynTree URDF exporter.
 */
struct URDFExporterOptions
{
    /**
     *
     * Specify the base link of the exported URDF.
     *
     * Differently from the iDynTree::Model class, the URDF file format represent the multibody structure
     * using a directed tree representation, for which it is necessary to
     *
     * If this string is empty (default value), the default base link contained in the model
     * will be used.
     *
     * Default value: ""
     */
    std::string baseLink;

    /**
     * If true, all the "additional frames"
     * available will be added to the URDF as "fake links",
     * because the URDF file specification does not support the concept of frames.
     * In particular the frames with name FRAME will be added to their link via
     * a fake fixed joint named FRAME_fixed_joint
     *
     * Default value: true
     */
    bool exportFrames;

    /**
     * Constructor, containing default values.
     */
    URDFExporterOptions(): baseLink(""),
                         exportFrames(true)
    {
    }
};

/**
 * \ingroup iDynTreeModelIO
 *
 * Export a iDynTree::Model object to a URDF file.
 *
 * @see URDFExporterOptions for more details on supported options.
 *
 * @warning This function does not support exporting sensor or solid shapes at the moment.
 *
 * @param[in] options the URDFParserOptions struct of options passed to the parser
 * @return true if all went ok, false otherwise.
 */
bool URDFFromModel(const iDynTree::Model & model,
                   const std::string & urdf_filename,
                   const URDFExporterOptions options=URDFExporterOptions());

/**
 * \ingroup iDynTreeModelIO
 *
 * Export a iDynTree::Model object to a URDF string.
 *
 * @see URDFExporterOptions for more details on supported options.
 *
 * @warning This function does not support exporting sensor or solid shapes at the moment.
 *
 * @param[in] options the URDFParserOptions struct of options passed to the parser
 * @return true if all went ok, false otherwise.
 */
bool URDFStringFromModel(const iDynTree::Model & output,
                         std::string & urdf_string,
                         const URDFExporterOptions options=URDFExporterOptions());


}

#endif
