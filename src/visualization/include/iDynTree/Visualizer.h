/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef IDYNTREE_VISUALIZER_H
#define IDYNTREE_VISUALIZER_H

#include <string>

#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/LinkState.h>

namespace iDynTree
{
class Model;
class Transform;
class Visualizer;

/**
 * Interface to manipulate the camera parameters.
 */
class ICamera
{
public:
    /**
      * Destructor
      */
    virtual ~ICamera() = 0;

    /**
     * Set the linear position of the camera w.r.t to the world.
     */
    virtual void setPosition(const iDynTree::Position & cameraPos) = 0;

    /**
     * Set the target of the camera (i.e. the point the camera is looking into) w.r.t. the world.
     */
    virtual void setTarget(const iDynTree::Position & cameraPos) = 0;
};

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

/**
 * Visualizer options
 */
struct VisualizerOptions
{
    /**
     * Set the visualizer to be verbose, useful for debug (default : false).
     */
    bool verbose;

    VisualizerOptions(): verbose(false)
    {
    }
};

/**
 * Class to visualize a set of iDynTree models
 */
class Visualizer
{
friend class ModelVisualization;

private:
    struct VisualizerPimpl;
    VisualizerPimpl * pimpl;

    // Disable copy for now
    Visualizer(const Visualizer& other);
    Visualizer& operator=(const Visualizer& other);
public:
    Visualizer();
    virtual ~Visualizer();

    /**
     * Initialize the visualization.
     *
     * \note this is called implicitly when addModel is called for the first time.
     */
    bool init(const VisualizerOptions = VisualizerOptions());

    /**
     * Get number of models visualized.
     */
    size_t getNrOfVisualizedModels();

    /**
     *
     */
    std::string getModelInstanceName(size_t modelInstanceIndex);

    /**
     * Get the index of a given model instance, or -1 if there is not such model instance.
     */
    int getModelInstanceIndex(const std::string instanceName);

    /**
     * Add an instance of a given model to the visualization.
     *
     * @param[in] model
     * @param[in] instanceName name of the instance of the model added.
     * @return true if all went well, false otherwise.
     */
    bool addModel(const iDynTree::Model & model,
                  const std::string & instanceName);

    /**
     * Return an interface to a visualization of a model.
     *
     * \note the modelIdx is invalidated whenever a model is removed from the visualization.
     *
     * @return a reference to a valid ModelVisualization if instanceName is the name of a model instance.
     */
    ModelVisualization& modelViz(size_t modelIdx);

    /**
     * Return an interface to a visualization of a model.
     *
     * @return a reference to a valid ModelVisualization if instanceName is the name of a model instance.
     */
    ModelVisualization& modelViz(const std::string & instanceName);

    /**
     * Return an interface to manipulate the camera in the visualization.
     */
    ICamera& camera();

    /**
     * Wrap the run method of the Irrlicht device.
     */
    bool run();

    /**
     * Draw the visualization.
     */
    void draw();

    /**
     * Right the current visualization to a image file.
     *
     * The format of the image is desumed from the filename.
     *
     * For more info on the process of writing the image,
     * check irr::video::IVideoDriver::writeImageToFile irrlicht method.
     *
     * @return true if all went ok, false otherwise.
     */
    bool drawToFile(const std::string filename="iDynTreeVisualizerScreenshot.png");

    /**
     * Close the visualizer.
     */
    void close();
};

}

#endif
