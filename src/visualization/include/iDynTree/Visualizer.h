/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef IDYNTREE_VISUALIZER_H
#define IDYNTREE_VISUALIZER_H

#include <string>

#include <iDynTree/Core/Direction.h>

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

    /**
     * Set the up vector of the camera w.r.t to the world.
     */
    virtual void setUpVector(const Direction& upVector) = 0;
};

/**
 * Basic structure to encode color information
 */
class ColorViz
{
public:
    /**
     * Red component of the color.
     */
    float r;

    /**
     * Green component of the color.
     */
    float g;

    /**
     * Blue component of the color.
     */
    float b;

    /**
     * Alpha component of the color.
     */
    float a;

    /**
     * Default constructor (to white)
     */
    ColorViz();

    /**
     * Constructor for rgba.
     */
    ColorViz(float r, float g, float b, float a);

    /**
     * Build a color from a Vector4 rgba.
     */
    ColorViz(const Vector4 & rgba);
};

enum LightType
{
    //! point light, it has a position in space and radiates light in all directions
    POINT_LIGHT,
    //! directional light, coming from a direction from an infinite distance
    DIRECTIONAL_LIGHT
};

/**
 * Interface to a light visualization.
 */
class ILight
{
public:
    /**
     * Denstructor
     */
    virtual ~ILight() = 0;

    /**
     * Get the name.
     */
    virtual const std::string & getName() const = 0;

    /**
     * Set the light type.
     */
    virtual void setType(const LightType type) = 0;

    /**
     * Get the light type.
     */
    virtual LightType getType() = 0;

    /**
     * Set the linear position of the light w.r.t to the world.
     */
    virtual void setPosition(const iDynTree::Position & cameraPos) = 0;

    /**
     * Get the linear position of the light w.r.t to the world.
     */
    virtual iDynTree::Position getPosition() = 0;

    /**
     * Set the light direction (only meaningful if the light is DIRECTIONAL_LIGHT).
     */
    virtual void setDirection(const Direction& lightDirection) = 0;

    /**
     * Get the light direction (only meaningful if the light is DIRECTIONAL_LIGHT).
     */
    virtual Direction getDirection() = 0;

    /**
     * Set ambient color of the light.
     */
    virtual void setAmbientColor(const ColorViz & ambientColor) = 0;

    /**
     * Get ambient color of the light.
     */
    virtual ColorViz getAmbientColor() = 0;

    /**
     * Set specular color of the light.
     */
    virtual void setSpecularColor(const ColorViz & ambientColor) = 0;

    /**
     * Get specular color of the light.
     */
    virtual ColorViz getSpecularColor() = 0;

    /**
     * Set ambient color of the light.
     */
    virtual void setDiffuseColor(const ColorViz & ambientColor) = 0;

    /**
     * Get ambient color of the light.
     */
    virtual ColorViz getDiffuseColor() = 0;
};

/**
 * Interface to manipulate the elements in the enviroment (background, root frame, reference lines)
 */
class IEnvironment
{
public:
    /**
     * Denstructor
     */
    virtual ~IEnvironment() = 0;

    /**
     * Get the list of the elements in the enviroment.
     *
     * The function returns the following list:
     *  * floor_grid
     *  * world_frame
     */
    virtual std::vector<std::string> getElements() = 0;

    /**
     *
     * @return true if the visibility is correctly setted, false otherwise.
     */
    virtual bool setElementVisibility(const std::string elementKey, bool isVisible) = 0;

    /**
     * Set the background color.
     */
    virtual void setBackgroundColor(const ColorViz & backgroundColor) = 0;

    /**
     * Set the ambient light of the enviroment.
     */
    virtual void setAmbientLight(const ColorViz & ambientLight) = 0;

    /**
     * Get the list of lights present in the visualization.
     */
    virtual std::vector<std::string> getLights() = 0;

    /**
     * Add a light.
     */
    virtual bool addLight(const std::string & lightName) = 0;

    /**
     * Return an interface to a light.
     */
    virtual ILight & lightViz(const std::string & lightName) = 0;

    /**
     * Remove a light from visualization.
     *
     * @return true if the light was present and was removed, false otherwise.
     */
    virtual bool removeLight(const std::string & lightName) = 0;
};



/**
 * Interface to the visualization of a model istance.
 */
class IModelVisualization
{
public:
    /**
     * Denstructor
     */
    virtual ~IModelVisualization() = 0;

    /**
     * Set the position of the model (using base position and joint positions)
     */
    virtual bool setPositions(const Transform & world_H_base, const VectorDynSize & jointPos) = 0;

    /**
     * Set the positions of the model by directly specifing link positions wrt to the world.
     */
    virtual bool setLinkPositions(const LinkPositions & linkPos) = 0;

    /**
     * Reference to the used model.
     */
    virtual Model & model() = 0;

    /**
     * Get the instance name.
     */
    virtual std::string getInstanceName() = 0;
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
    IModelVisualization& modelViz(size_t modelIdx);

    /**
     * Return an interface to a visualization of a model.
     *
     * @return a reference to a valid ModelVisualization if instanceName is the name of a model instance.
     */
    IModelVisualization& modelViz(const std::string & instanceName);

    /**
     * Return an interface to manipulate the camera in the visualization.
     */
    ICamera& camera();

    /**
     * Return an interface to manipulate the visualization environment.
     */
    IEnvironment& enviroment();

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
