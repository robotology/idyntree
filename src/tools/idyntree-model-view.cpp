/**
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <iDynTree/Visualizer.h>

#include "cmdline.h"

#include <iostream>
#include <cstdlib>

/**
 * Add the option supported by the idyntree-model-info utility
 */
void addOptions(cmdline::parser &cmd)
{
    cmd.add<std::string>("model", 'm',
                         "Model to load.",
                         true);
}

int main(int argc, char** argv)
{
    cmdline::parser cmd;
    addOptions(cmd);
    cmd.parse_check(argc, argv);

    // Read model
    std::string modelPath = cmd.get<std::string>("model");
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(modelPath);

    if( !ok )
    {
        std::cerr << "Impossible to read model at file " << modelPath << std::endl;
        return EXIT_FAILURE;
    }

    // Visualize the model
    iDynTree::Visualizer visualizer;
    ok = visualizer.addModel(mdlLoader.model(),"model");

    if( !ok )
    {
        std::cerr << "Impossible to read visualizer." << std::endl;
        return EXIT_FAILURE;
    }

    // Visualization loop
    while( visualizer.run() )
    {
        visualizer.draw();
    }

    return EXIT_SUCCESS;
}


