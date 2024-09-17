// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_VISUALIZER_H
#define IDYNTREE_VISUALIZER_H

#include <string>
#include <vector>
#include <utility>

#include <iDynTree/Direction.h>
#include <iDynTree/Position.h>

#include <iDynTree/JointState.h>
#include <iDynTree/LinkState.h>

#include <iDynTree/SolidShapes.h>

namespace iDynTree
{
class Model;
class Transform;
class Visualizer;

/**
 * Interface to animate the camera and control it via the mouse
 */
class ICameraAnimator
{
public:

    /**
     * Enable the control of camera via the mouse.
     * By default, left button rotates the camera, the right button translates it on the plane.
     * The middle button, translates the camera up and down, while the wheel increases or decreases the zoom.
     */
    virtual void enableMouseControl(bool enabled = true) = 0;

    /**
     *  Returns the speed of movement
     */
    virtual double getMoveSpeed() const = 0;

    /**
     *  Sets the speed of movement
     */
    virtual void setMoveSpeed(double moveSpeed) = 0;

    /**
     *  Returns the rotation speed
     */
    virtual double getRotateSpeed() const = 0;

    /**
     *  Set the rotation speed
     */
    virtual void setRotateSpeed(double rotateSpeed) = 0;

    /**
     *  Returns the zoom speed
     */
    virtual double getZoomSpeed() const = 0;

    /**
     *  Set the zoom speed
     */
    virtual void setZoomSpeed(double zoomSpeed) = 0;
};

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
     * Get the linear position of the camera w.r.t to the world.
     */
    virtual iDynTree::Position getPosition() = 0;

    /**
     * Get the target of the camera (i.e. the point the camera is looking into) w.r.t. the world.
     */
    virtual iDynTree::Position getTarget() = 0;

    /**
     * Set the up vector of the camera w.r.t to the world.
     */
    virtual void setUpVector(const Direction& upVector) = 0;

    /**
     * Get a pointer to the CameraAnimator object.
     * It is not supposed to be deallocated, and its lifespan coincides with the one of the ICamera
     * It is accessible only after the visualizer has been initialized.
     */
    virtual ICameraAnimator* animator() = 0;

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

    /**
     * Return as a Vector4.
     */
    Vector4 toVector4() const;
};

/**
 * Basic structure to encode pixel information
 */
class PixelViz : public ColorViz
{
public:

    /**
     * Width position of the pixel.
     */
    unsigned int width;

    /**
     * Height position of the pixel.
     */
    unsigned int height;

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
     *
     * | Element name  | Description |
     * |:-------------:|:-----------:|
     * |  floor_grid   | Grid used to indicated the z = 0 plane. |
     * |  world_frame  | XYZ (RBG) arrows indicating the world frame. |
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
     * Set the floor grid color.
     */
    virtual void setFloorGridColor(const ColorViz & floorGridColor) = 0;

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
 * Interface to the visualization of jets attached to a model.
 */
class IJetsVisualization
{
public:
    /**
     * Denstructor
     */
    virtual ~IJetsVisualization() = 0;

    /**
     * Set the frame on the model on which the jets are visualized.
     *
     * @note this will delete any state related to jets visualization.
     */
    virtual bool setJetsFrames(const std::vector< std::string > & jetsFrames) = 0;

    /**
     * Get the number of visualized jets.
     *
     */
    virtual size_t getNrOfJets() const = 0;

    /**
     * Get jet direction.
     */
    virtual Direction getJetDirection(const int jetIndex) const = 0;

    /**
     * Set jet direction.
     */
    virtual bool setJetDirection(const int jetIndex, const Direction & jetDirection) = 0;

    /**
     * Set jet color.
     */
    virtual bool setJetColor(const int jetIndex, const ColorViz & jetColor) = 0;

    /**
     * The jets are visualized as cones attached to the frame,
     */
    virtual bool setJetsDimensions(const double & minRadius, const double & maxRadius, const double & maxLenght) = 0;

    /**
     * Set the jets intensity.
     *
     * @param[in] jetsIntensity a vector of getNrOfJets values, from 0 (no thrust) 1 (max thrust).
     *
     */
    virtual bool setJetsIntensity(const VectorDynSize & jetsIntensity) = 0;
};

/**
 * The interface to add a label in the visualizer.
 */

class ILabel
{
public:

    /**
     * Destructor
     */
    virtual ~ILabel() = 0;

    /**
     * @brief Set the text of the label
     * @param text The text to be displayed
     */
    virtual void setText(const std::string& text) = 0;

    /**
     * @brief Get the label text
     * @return The text shown in the label.
     */
    virtual std::string getText() const = 0;

    /**
     * @brief Set the height of the label. The width is computed automatically
     * @param height The height to be set
     */
    virtual void setSize(float height) = 0;

    /**
     * @brief Set the width and the height of the label
     * @param width The width of the label
     * @param height The height of the label
     */
    virtual void setSize(float width, float height) = 0;

    /**
     * @brief Get the width of the label
     * @return The width of the label
     */
    virtual float width() const = 0;

    /**
     * @brief Get the height of the label
     * @return The height of the label
     */
    virtual float height() const = 0;

    /**
     * @brief Set the position of the label
     * @param position The position of the label
     */
    virtual void setPosition(const iDynTree::Position& position) = 0;

    /**
     * @brief Get the position of the label
     * @return The position of the label
     */
    virtual iDynTree::Position getPosition() const = 0;

    /**
     * @brief Set the color of the label
     * @param color The color of the label
     */
    virtual void setColor(const iDynTree::ColorViz& color) = 0;

    /**
     * @brief Set the visibility of the label
     * @param visible The visibility of the label
     */
    virtual void setVisible(bool visible = true) = 0;
};

/**
 * Interface to the visualization of vectors.
 */
class IVectorsVisualization
{
public:
    /**
     * Denstructor
     */
    virtual ~IVectorsVisualization() = 0;

    /**
     * @brief Add a vector in the visualization
     * @return The vector index.
     */
    virtual size_t addVector(const Position & origin, const Direction & direction, double modulus) = 0;

    /**
     * @brief Add a vector in the visualization
     * @return The vector index.
     */
    virtual size_t addVector(const Position & origin, const Vector3 & components) = 0;

    /**
     * Get the number of visualized vectors.
     *
     */
    virtual size_t getNrOfVectors() const = 0;

    /**
     * Get vector properties.
     */
    virtual bool getVector(size_t vectorIndex, Position & currentOrigin,
                           Direction & currentDirection, double & currentModulus) const = 0;

    /**
     * Get vector properties.
     */
    virtual bool getVector(size_t vectorIndex, Position & currentOrigin, Vector3 & components) const = 0;

    /**
     * Update Vector
     */
    virtual bool updateVector(size_t vectorIndex, const Position & origin, const Direction & direction, double modulus) = 0;

    /**
     * Update Vector
     */
    virtual bool updateVector(size_t vectorIndex, const Position & origin, const Vector3& components) = 0;

    /**
     * Set vector color.
     */
    virtual bool setVectorColor(size_t vectorIndex, const ColorViz & vectorColor) = 0;

    /**
     * Set the default color for the vector.
     */
    virtual void setVectorsDefaultColor(const ColorViz &vectorColor) = 0;

    /**
     * Set the color for all the existing vectors.
     */
    virtual void setVectorsColor(const ColorViz &vectorColor) = 0;

    /**
     * @brief Determines the dimension of the visualized arrows
     * @param zeroModulusRadius Constant offset for the arrow radius.
     * @param modulusMultiplier Multiplies the modulus and adds up to the zeroModulusRadius to get the total arrow radius.
     * @return true if successfull, false in case of negative numbers.
     */
    virtual bool setVectorsAspect(double zeroModulusRadius, double modulusMultiplier, double heightScale) = 0;

    /**
     * @brief Set the visibility of a given vector
     * @param The index of the vector to change the visibility
     * @param visible The visibility of the vector
     * @return true if successfull, false if the index is out of bounds
     */
    virtual bool setVisible(size_t vectorIndex, bool visible = true) = 0;

    /**
     * @brief Get the label associated to a vector
     * @param vectorIndex The index of the vector to get the label
     * @return The label associated to the vector. nullptr if the vectorIndex is out of bounds.
     */
    virtual ILabel* getVectorLabel(size_t vectorIndex) = 0;
};

/**
 * Interface to the visualization of frames.
 */
class IFrameVisualization
{
public:

    /**
     * Destructor
     */
    virtual ~IFrameVisualization() = 0;

    /**
     * Add a frame in the visualization
     * Returns the frame index.
     */
    virtual size_t addFrame(const Transform& transformation, double arrowLength = 1.0) = 0;

    /**
     * Set the specified frame visible or not.
     * Returns true in case of success, false otherwise (for example if the frame does not exists).
     */
    virtual bool setVisible(size_t frameIndex, bool isVisible) = 0;

    /**
     * Get the number of visualized frames.
     *
     */
    virtual size_t getNrOfFrames() const = 0;

    /**
     * Get frame transform, relative to the parent frame (world if the frame is attached to the world).
     */
    virtual bool getFrameTransform(size_t frameIndex, Transform& currentTransform) const = 0;

    /**
     * Update Frame, the transformation is relative to the parent frame (world if the frame is attached to the world).
     */
    virtual bool updateFrame(size_t frameIndex, const Transform& transformation) = 0;

    /**
    * Get the parent of a frame.
    * Returns a pair with the first element being the model name, and the second the frame name to which it is attached.
    * If the frame is attached to the world, both elements are empty strings.
    */
    virtual std::pair<std::string, std::string> getFrameParent(size_t frameIndex) const = 0;

    /**
    * Set the parent of a frame.
    * Returns true in case of success, false otherwise (for example if the frame index is out of bounds).
    * If the modelName and frameName are empty strings, the frame is attached to the world.
    * If the model name is specified, but not the frame name, it is attached to the root link of the model.
    */
    virtual bool setFrameParent(size_t frameIndex, const std::string& modelName, const std::string& frameName) = 0;

    /**
     * Get the label of a frame.
     *
     * Returns nullptr of the frame index is out of bounds.
     */
    virtual ILabel* getFrameLabel(size_t frameIndex) = 0;
};

/**
 * Interface to the visualization of generic solid shapes.
 */
 class IShapeVisualization
 {
public:
        /**
        * Destructor
        */
        virtual ~IShapeVisualization() = 0;

        /**
        * Add a shape in the visualization.
        * If the modelName and linkName are specified, the shape is attached to the specific frame.
        * If they are not specified, or cannot be found, the shape is attached to the world.
        * If the model name is specified, but not the frame name, it is attached to the root link of the model.
        * The initial transform is specified by the shape itself (Link_H_geometry).
        * Returns the shape index.
        */
        virtual size_t addShape(const iDynTree::SolidShape& shape,
                                const std::string& modelName = "",
                                const std::string& frameName = "") = 0;

        /**
        * Set the specified shape visible or not.
        * Returns true in case of success, false otherwise (for example if the shape does not exists).
        */
        virtual bool setVisible(size_t shapeIndex, bool isVisible) = 0;

        /**
        * Get the number of visualized shapes.
        *
        */
        virtual size_t getNrOfShapes() const = 0;

        /**
        * Get shape transform with respect the parent frame (world if the shape is attached to the world).
        */
        virtual bool getShapeTransform(size_t shapeIndex, Transform& currentTransform) const = 0;

        /**
        * Set the shape transform with respect the parent frame (world if the shape is attached to the world).
        */
        virtual bool setShapeTransform(size_t shapeIndex, const Transform& transformation) = 0;

        /**
        * Set the color of the shape.
        * Returns true in case of success, false otherwise (for example if the shape does not exists).
        */
        virtual bool setShapeColor(size_t shapeIndex, const ColorViz& shapeColor) = 0;

        /**
        * Change the shape.
        * The previous shape is removed.
        * Returns true in case of success, false otherwise (for example if the shape index is out of bounds).
        */
        virtual bool changeShape(size_t shapeIndex, const iDynTree::SolidShape& newShape) = 0;


        /**
        * Get the parent of a shape.
        * Returns a pair with the first element being the model name, and the second the frame name.
        * If the shape is attached to the world, both elements are empty strings.
        */
        virtual std::pair<std::string, std::string> getShapeParent(size_t shapeIndex) const = 0;

        /**
        * Set the parent of a shape.
        * Returns true in case of success, false otherwise (for example if the shape index is out of bounds).
        * If the modelName and frameName are empty strings, the shape is attached to the world.
        * If the model name is specified, but not the frame name, it is attached to the root link of the model.
        */
        virtual bool setShapeParent(size_t shapeIndex, const std::string& modelName, const std::string& frameName) = 0;

        /**
        * Get the label of a shape.
        *
        * Returns nullptr of the shape index is out of bounds.
        */
        virtual ILabel* getShapeLabel(size_t shapeIndex) = 0;
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

    /**
     * Set the visibility of all the link of the model.
     */
    virtual void setModelVisibility(const bool isVisible) = 0;

    /**
     * Set the color of all the geometries of the model.
     *
     * This will overwrite the material of the model, but it can be
     * reset by resetModelColor.
     */
    virtual void setModelColor(const ColorViz & modelColor) = 0;

    /**
     * Reset the colors of the model.
     */
    virtual void resetModelColor() = 0;

    /**
     * Set the color of all the geometries of the given link.
     *
     * This will overwrite the material of the link, but it can be
     * reset by resetLinkColor.
     */
    virtual bool setLinkColor(const LinkIndex& linkIndex, const ColorViz& linkColor) = 0;

    /**
     * Set the color of a single visual of a given link.
     *
     * This will overwrite the material of the visual, but it can be
     * reset by resetLinkColor.
     */
    virtual bool setVisualColor(const LinkIndex& linkIndex, const std::string& visualName, const ColorViz& visualColor) = 0;

    /**
     * Set the transparency of a given link of the model.
     *
     * This will overwrite the material of the link, but it can be
     * reset by resetLinkColor.
     */
    virtual bool setLinkTransparency(const LinkIndex& linkIndex, const double transparency) = 0;

    /**
     * Set the transparency of all the links of the model.
     *
     * This will overwrite the material of the links, but they can be
     * reset by resetLinkColor.
     */
    virtual void setModelTransparency(const double transparency) = 0;

    /**
     * Reset the colors of given link.
     */
    virtual bool resetLinkColor(const LinkIndex& linkIndex) = 0;

    /**
     * Get the name of the link in the model.
     */
    virtual std::vector< std::string > getLinkNames() = 0;

    /**
     * Set a given link visibility.
     */
    virtual bool setLinkVisibility(const std::string & linkName, bool isVisible) = 0;

    /**
     * Get list of visualization features that can be enabled/disabled.
     *
     * This method will return the follow list:
     * | Visualization  feature | Description                              | Default value |
     * |:----------------------:|:----------------------------------------:|:-------------:|
     * |  wireframe             | Visualize mesh of the model as wireframe | false  |
     */
    virtual std::vector<std::string> getFeatures() = 0;

    /**
     * @return true if the visibility is correctly setted, false otherwise.
     */
    virtual bool setFeatureVisibility(const std::string& elementKey, bool isVisible) = 0;

    /**
     * Get a reference to the internal IJetsVisualization interface.
     */
    virtual IJetsVisualization& jets() = 0;

    /**
     * Get the transformation of given link with respect to visualizer world \f$ {}^w wH_{link}\f$
     */
    virtual Transform getWorldLinkTransform(const LinkIndex& linkIndex) = 0;

    /**
     * Get the transformation of given link with respect to visualizer world \f$ {}^w H_{link}\f$
     */
    virtual Transform getWorldLinkTransform(const std::string& linkName) = 0;

    /**
     * Get the transformation of given frame with respect to visualizer world \f$ {}^w H_{frame}\f$
     */
    virtual Transform getWorldFrameTransform(const FrameIndex& frameIndex) = 0;

    /**
     * Get the transformation of given frame with respect to visualizer world \f$ {}^w H_{frame}\f$
     */
    virtual Transform getWorldFrameTransform(const std::string& frameName) = 0;

    /**
     * Get the label for the model.
     */
    virtual ILabel& label() = 0;
};

/**
 * The interface for an object that can be used as an additional target for the renderer.
 * This allows rendering the scene using dimensions and environment that are different from
 * the main window. The camera is in common. Any camera change in the main window is also
 * reflected in the other textures.
 */
class ITexture
{
public:

    /**
     * Destructor
     */
    virtual ~ITexture() = 0;

    /**
     * Return an interface to manipulate the texture environment.
     */
    virtual IEnvironment& environment() = 0;

    /**
     * @brief Get the color of the pixel at the given position in the additional texture.
     *
     * Remember to call draw() first.
     * @param width The width of the pixel
     * @param height The height of the pixel
     * @return The color of the pixel
     */
    virtual ColorViz getPixelColor(unsigned int width, unsigned int height) const = 0;

    /**
     * @brief Get the pixels of the texture.
     *
     * Remember to call draw() first.
     * @param pixels The output pixels. The size of the vector will be equal to the total number of
     * pixels of the rendered texture, i.e. width of the texture times its height. Both these two parameters
     * can be set in the textureOptions passed to the method ITexturesHandler::add. The pixels are saved in
     * col-major format.
     * @note This operation may affect the time performances of the visualizer, especially if the texture is large.
     * @return True in case of success, false otherwise
     */
    virtual bool getPixels(std::vector<PixelViz>& pixels) const = 0;

    /**
     * Draw the current texture to a image file.
     *
     * The format of the image is desumed from the filename.
     *
     * For more info on the process of writing the image,
     * check irr::video::IVideoDriver::writeImageToFile irrlicht method.
     *
     * @return true if all went ok, false otherwise.
     */
    virtual bool drawToFile(const std::string filename="iDynTreeVisualizerTextureScreenshot.png") const = 0;

    /**
     * @brief Enable/disable the drawing on the texture.
     * @param enabled If true (default), the visualizer will draw on the texture when calling draw();
     */
    virtual void enableDraw(bool enabled = true) = 0;

    /**
     * Get the texture width.
     */
    virtual int width() const = 0;

    /**
     * Get the texture height.
     */
    virtual int height() const = 0;

    /**
     * Set the area used for drawing operations. The entirety of the texture is cleared between two draw() calls.
     * Use this in conjunction with subDraw() to draw on different parts of the same texture.
     * Use call it with (0, 0, width(), height()) to draw on the full texture (this is done by default).
     */
    virtual bool setSubDrawArea(int xOffsetFromTopLeft, int yOffsetFromTopLeft, int subImageWidth, int subImageHeight) = 0;

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

    /**
     * Initial width (in pixels) of the created windows (default: 800).
     */
    int winWidth;

    /**
     * Initial height (in pixels) of the created window (default: 600).
     */
    int winHeight;

    /**
     * Dimension of the root frame arrows in meters (default: 1.0).
     */
    double rootFrameArrowsDimension;

    VisualizerOptions(): verbose(false),
                         winWidth(800),
                         winHeight(600),
                         rootFrameArrowsDimension(1.0)
    {
    }
};

class ITexturesHandler
{
public:

    /**
     * Destructor
     */
    virtual ~ITexturesHandler() = 0;

    /**
     * @brief Add a texture
     * @param The name of the texture
     * @param visualizerOptions The options for the texture
     * @note The pointer should not be deleted. Its lifespan coincides with the one of the ITextureHandler.
     * @return A ITexture pointer in case of success. A nullptr otherwise
     */
    virtual ITexture* add(const std::string& name, const VisualizerOptions& textureOptions = VisualizerOptions()) = 0;

    /**
     * @brief Get a specific texture
     * @param The name of the texture to get.
     * @note The pointer should not be deleted. Its lifespan coincides with the one of the ITextureHandler.
     * @return the pointer to the texture. A nullptr if that texture does not exists.
     */
    virtual ITexture* get(const std::string& name) = 0;
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
    bool init(const VisualizerOptions& visualizerOptions = VisualizerOptions());

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
    IDYNTREE_DEPRECATED_WITH_MSG("Please use the environment method instead (this method has a typo).")
    IEnvironment& enviroment();

    /**
     * Return an interface to manipulate the visualization environment.
     */
    IEnvironment& environment();

    /**
     * Get a reference to the internal IVectorsVisualization interface.
     */
    IVectorsVisualization& vectors();

    /**
     * Get a reference to the internal IFrameVisualization interface.
     */
    IFrameVisualization& frames();

    /**
     * Get a reference to the internal ITexturesHandler interface.
     */
    ITexturesHandler& textures();

    /**
     * Get a reference to the internal IShapeVisualization interface.
     */
    IShapeVisualization& shapes();

    /**
     * Get a label given a name. Note: this does not set the text in the label.
     */
    ILabel& getLabel(const std::string& labelName);

    /**
     * Get the visualizer width. Note: if the window is resized, this returns the correct size only if run is called first.
     */
    int width() const;

    /**
     * Get the visualizer height. Note: if the window is resized, this returns the correct size only if run is called first.
     */
    int height() const;

    /**
     * Wrap the run method of the Irrlicht device.
     */
    bool run();

    /**
     * Draw the visualization.
     */
    void draw();

    /**
     * Draw the visualization in a subportion of the window. Need to call draw once done.
     */
    void subDraw(int xOffsetFromTopLeft, int yOffsetFromTopLeft, int subImageWidth, int subImageHeight);

    /**
     * Draw the current visualization to a image file.
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

    /**
     * @brief Get if the visualizer window is active (to allow drawing only if necessary)
     * @return True if the window is active, false otherwise.
     */
    bool isWindowActive() const;

    /**
     * @brief Set the color palette.
     * @param name name of the color palette. Currently only vanilla and meshcat are supported.
     * @return true if all went ok, false otherwise.
     */
    bool setColorPalette(const std::string& name);
};

}

#endif
