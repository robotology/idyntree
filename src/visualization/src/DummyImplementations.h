/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_DUMMYIMPLEMENTATIONS_H
#define IDYNTREE_DUMMYIMPLEMENTATIONS_H

#include <iDynTree/Visualizer.h>
#include <iDynTree/Model/Model.h>

namespace iDynTree
{

/**
 * Dummy camera.
 */
class DummyCamera : public ICamera
{
public:
    virtual ~DummyCamera() {};

    virtual void setPosition(const iDynTree::Position &) {};
    virtual void setTarget(const iDynTree::Position &) {};
    virtual void setUpVector(const Direction&) {};
};

/**
 * Dummy light.
 */
class DummyLight : public ILight
{
    std::string dummyName;
public:
    DummyLight() { dummyName = "invalidLight"; }
    virtual ~DummyLight() {}
    const std::string & getName() const { return dummyName; }
    virtual void setType(const LightType) {}
    virtual LightType getType() { return POINT_LIGHT; }
    virtual void setPosition(const iDynTree::Position &) {}
    virtual iDynTree::Position getPosition() { return Position::Zero(); }
    virtual void setDirection(const Direction&) {}
    virtual Direction getDirection() { return Direction::Default(); }
    virtual void setAmbientColor(const ColorViz &) {}
    virtual ColorViz getAmbientColor() { return ColorViz(); }
    virtual void setSpecularColor(const ColorViz &) {}
    virtual ColorViz getSpecularColor() { return ColorViz(); }
    virtual void setDiffuseColor(const ColorViz & ) {}
    virtual ColorViz getDiffuseColor()  { return ColorViz(); }
};

/**
 * Dummy environment.
 */
class DummyEnvironment : public IEnvironment
{
    DummyLight m_dummyLight;
public:
    virtual ~DummyEnvironment() {};
    virtual bool setElementVisibility(const std::string /*elementKey*/, bool /*isVisible*/) {return false;}
    virtual std::vector< std::string > getElements() {  return std::vector< std::string >(); }
    virtual void setBackgroundColor(const ColorViz & ) {}
    virtual void setAmbientLight(const ColorViz &) {};
    virtual std::vector<std::string> getLights() { return std::vector<std::string>(); }
    virtual bool addLight(const std::string &) { return false; }
    virtual ILight & lightViz(const std::string &) { return m_dummyLight; }
    virtual bool removeLight(const std::string &) { return false; }
};

/**
 * Dummy model visualization.
 */
class DummyModelVisualization : public IModelVisualization
{
    Model m_dummyModel;
public:
    virtual ~DummyModelVisualization() {};
    virtual bool init(const Model& , const std::string , Visualizer &) { return false; }
    virtual bool setPositions(const Transform & , const VectorDynSize & ) { return false; }
    virtual bool setLinkPositions(const LinkPositions & ) { return false; }
    virtual Model & model() { return m_dummyModel; }
    virtual void close() {}
    virtual std::string getInstanceName() { return "dummyModelVisualizationInstance"; }
    virtual void setModelVisibility(const bool) {}
    virtual void setModelColor(const ColorViz & ) {}
    virtual void resetModelColor() {}
    virtual std::vector< std::string > getLinkNames() { return std::vector<std::string>(); };
    virtual bool setLinkVisibility(const std::string &, bool) { return false; }
    virtual std::vector<std::string> getFeatures() { return std::vector<std::string>(); }
    virtual bool setFeatureVisibility(const std::string& , bool) { return false; }

};

}

#endif
