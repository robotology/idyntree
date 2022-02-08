/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef IDYNTREE_URDF_MODEL_EXPORT_H
#define IDYNTREE_URDF_MODEL_EXPORT_H

#include <string>

#include <iDynTree/ModelIO/ModelExporter.h>

namespace iDynTree

{

class Model;

/**
 * \ingroup iDynTreeModelIO
 *
 * Export a iDynTree::Model object to a URDF file.
 *
 * @see iDynTree::ModelExporterOptions for more details on supported and default options.
 *
 * @warning This function does not support exporting sensor or solid shapes at the moment.
 *
 * @param[in] urdf_filename Path to the URDF file that will be created.
 *                          It can be either a relative filename with respect to the current working directory,
 *                          or an absolute filename.
 * @param[in] options the iDynTree::ModelExporterOptions struct of options passed to the parser.
 * @return true if all went ok, false otherwise.
 */
bool URDFFromModel(const iDynTree::Model & model,
                   const std::string & urdf_filename,
                   const ModelExporterOptions options=ModelExporterOptions());

/**
 * \ingroup iDynTreeModelIO
 *
 * Export a iDynTree::Model object to a URDF string.
 *
 * @see iDynTree::ModelExporterOptions for more details on supported and default options.
 *
 * @warning This function does not support exporting sensor or solid shapes at the moment.
 *
 * @param[in] options the iDynTree::ModelExporterOptions struct of options passed to the parser.
 * @return true if all went ok, false otherwise.
 */
bool URDFStringFromModel(const iDynTree::Model & output,
                         std::string & urdf_string,
                         const ModelExporterOptions options=ModelExporterOptions());


}

#endif
