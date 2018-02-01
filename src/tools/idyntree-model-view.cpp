/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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


