/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include <iDynTree/Model/Model.h>

// To load models from file
#include <iDynTree/ModelIO/ModelLoader.h>

// To approximate shapes of a model
#include <iDynTree/ModelTransformersSolidShapes.h>

// To write model to file
#include <iDynTree/ModelIO/ModelExporter.h>

#include "cmdline.h"

#include <iostream>
#include <cstdlib>

/**
 * Add the option supported by the idyntree-model-simplify-shapes utility
 */
void addOptions(cmdline::parser &cmd)
{
    cmd.add<std::string>("model", 'm',
                         "Input model to load and simplify.",
                         true);

    cmd.add<std::string>("output-model", 'o',
                         "Output simplified model.",
                         true);
}

int main(int argc, char** argv)
{
    cmdline::parser cmd;
    addOptions(cmd);
    cmd.parse_check(argc, argv);

    // Read model
    const std::string& modelPath = cmd.get<std::string>("model");
    const std::string& outputModelPath = cmd.get<std::string>("output-model");

    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(modelPath);

    if (!ok)
    {
        std::cerr << "Impossible to read model at file " << modelPath << std::endl;
        return EXIT_FAILURE;
    }

    // Simplify the model
    iDynTree::ApproximateSolidShapesWithPrimitiveShapeOptions options = 
        iDynTree::ApproximateSolidShapesWithPrimitiveShapeOptions();
    options.conversionType = iDynTree::ConvertSolidShapesWithEnclosingAxisAlignedBoundingBoxes;
    iDynTree::Model simplifiedModel;
    ok = approximateSolidShapesWithPrimitiveShape(mdlLoader.model(), simplifiedModel);

    if (!ok)
    {
        std::cerr << "Impossible to simplify  model loaded from file " << modelPath << std::endl;
        return EXIT_FAILURE;
    }


    // Export the model to file
    iDynTree::ModelExporter mdlExporter;
    iDynTree::ModelExporterOptions exportOptions = iDynTree::ModelExporterOptions();
    ok = mdlExporter.init(simplifiedModel, iDynTree::SensorsList(), exportOptions);
    ok = ok && mdlExporter.exportModelToFile(outputModelPath);

    if (!ok)
    {
        std::cerr << "Impossible to export simplified model at file " << outputModelPath << std::endl;
        return EXIT_FAILURE;
    }


    return EXIT_SUCCESS;
}
