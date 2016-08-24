/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef IDYNTREE_MODEL_CONTACT_WRENCHES_VISUALIZATION_H
#define IDYNTREE_MODEL_CONTACT_WRENCHES_VISUALIZATION_H

#include <string>

#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/ContactWrench.h>

namespace iDynTree
{
class Model;
class Transform;
class Visualizer;

/**
 * Interface to the visualization of the contact wrenches of a model istance.
 */
class ModelContactWrenchesVisualization
{
private:


public:
    ModelContactWrenchesVisualization();
    ~ModelContactWrenchesVisualization();

    /**
     * Initialize the class.
     */
    bool init(ModelVisualization & model);



    /**
     * Remove the model from the visualization.
     */
    void close();
};

}

#endif
