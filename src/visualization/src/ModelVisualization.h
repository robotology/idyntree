// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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

    /**
     * @param[in] linkIndex Index of the link
     * @param[in] color The color to assign to the specified visual
     * @param[in] visualName the name of the visual for which to change the color, or "" to set all the visual of the links
     * @return true if all went ok, false otherwise
     */
    bool changeVisualsColor(const LinkIndex& linkIndex, const ColorViz& color, const std::string& visualName = "");
public:
    ModelVisualization();
    ~ModelVisualization();
    bool init(const Model& model, const std::string instanceName, irr::scene::ISceneManager * sceneManager);
    void close();

    irr::scene::ISceneNode * getFrameSceneNode(const std::string& frameName);

    virtual bool setPositions(const Transform & world_H_base, const VectorDynSize & jointPos);
    virtual bool setLinkPositions(const LinkPositions & linkPos);
    virtual Model & model();
    virtual std::string getInstanceName();
    virtual void setModelVisibility(const bool isVisible);
    virtual void setModelColor(const ColorViz & modelColor);
    virtual void resetModelColor();
    virtual bool setLinkColor(const LinkIndex& linkIndex, const ColorViz& linkColor);
    virtual bool setVisualColor(const LinkIndex& linkIndex, const std::string& visualName, const ColorViz& visualColor);
    virtual bool resetLinkColor(const LinkIndex& linkIndex);
    virtual std::vector< std::string > getLinkNames();
    virtual bool setLinkVisibility(const std::string & linkName, bool isVisible);
    virtual std::vector<std::string> getFeatures();
    virtual void setModelTransparency(const double transparency);
    virtual bool setLinkTransparency(const LinkIndex& linkIndex, const double transparency);
    virtual bool setFeatureVisibility(const std::string & elementKey, bool isVisible);
    void setWireframeVisibility(bool isVisible);
    void setTransparent(bool isTransparent);
    virtual IJetsVisualization& jets();
    virtual Transform getWorldLinkTransform(const LinkIndex& linkIndex);
    virtual Transform getWorldFrameTransform(const FrameIndex& frameIndex);
    virtual Transform getWorldLinkTransform(const std::string& linkName);
    virtual Transform getWorldFrameTransform(const std::string& frameName);
    virtual ILabel& label();
};

}

#endif
