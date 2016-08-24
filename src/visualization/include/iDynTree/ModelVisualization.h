/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef IDYNTREE_MODEL_VISUALIZATION_H
#define IDYNTREE_MODEL_VISUALIZATION_H

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
 * Interface to the visualization of a model istance.
 */
class ModelVisualization
{
private:
    struct ModelVisualizationPimpl;
    ModelVisualizationPimpl * pimpl;

    // Disable copy for now
    ModelVisualization(const ModelVisualization& other);
    ModelVisualization& operator=(const ModelVisualization& other);
public:
    ModelVisualization();
    ~ModelVisualization();

    /**
     * Create the model in the visualization.
     */
    bool init(const Model& model, const std::string instanceName, Visualizer & visualizer);

    /**
     * Set the position of the model (using base position and joint positions)
     */
    bool setPositions(const Transform & world_H_base, const JointPosDoubleArray & jointPos);

    /**
     * Set the positions of the model by directly specifing link positions wrt to the world.
     */
    bool setLinkPositions(const LinkPositions & linkPos);

    /**
     * Enable/disable external wrenches visualization.
     *
     * @return true if the visualization was successfully enabled/disabled.
     */
    bool visualizeContactWrenches(bool visualize);

    /**
     * Set the contact wrenches to visualize.
     *
     * @return true if all went well, false otherwise.
     */
    bool setContactWrenches(const LinkContactWrenches & contactWrenches);

    /**
     * Reference to the used model.
     */
    Model & model();

    /**
     * Remove the model from the visualization.
     */
    void close();

    /**
     * Get the instance name.
     */
    std::string getInstanceName();
};

}

#endif
