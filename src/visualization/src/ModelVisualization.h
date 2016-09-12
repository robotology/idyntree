/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_MODEL_VISUALIZATION_H
#define IDYNTREE_MODEL_VISUALIZATION_H

#include <iDynTree/Visualizer.h>

#include <irrlicht.h>

namespace iDynTree
{

class ModelVisualization: public IModelVisualization
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

    bool init(const Model& model, const std::string instanceName, irr::scene::ISceneManager * sceneManager);
    bool setPositions(const Transform & world_H_base, const VectorDynSize & jointPos);
    bool setLinkPositions(const LinkPositions & linkPos);
    Model & model();
    void close();
    std::string getInstanceName();
};

}

#endif
