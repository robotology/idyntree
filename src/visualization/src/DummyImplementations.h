/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
    virtual iDynTree::Position getPosition() {return iDynTree::Position();};
    virtual iDynTree::Position getTarget() {return iDynTree::Position();};
    virtual ICameraAnimator* animator() {return nullptr;};
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
    virtual void setFloorGridColor(const ColorViz & ){};
};


/**
 * Dummy jets.
 */
class DummyJetsVisualization : public IJetsVisualization
{
public:
    virtual ~DummyJetsVisualization() {};
    virtual bool setJetsFrames(const std::vector< std::string > & ) { return false; };
    virtual size_t getNrOfJets() const  { return 0; };
    virtual Direction getJetDirection(const int ) const  { return Direction::Default(); };
    virtual bool setJetDirection(const int , const Direction & )  { return false; };
    virtual bool setJetColor(const int , const ColorViz & )  { return false; };
    virtual bool setJetsDimensions(const double & , const double & , const double & ) { return false; };
    virtual bool setJetsIntensity(const VectorDynSize & ) { return false; };
};

class DummyVectorsVisualization : public IVectorsVisualization {
public:
    virtual ~DummyVectorsVisualization() override { }
    virtual size_t addVector(const Position &, const Direction &, double) override { return 0; }
    virtual size_t addVector(const Position &, const Vector3 &) override { return 0; }
    virtual size_t getNrOfVectors() const override { return 0; }
    virtual bool getVector(size_t, Position &, Direction &, double &) const override { return false; }
    virtual bool getVector(size_t, Position &, Vector3 &) const override { return false; }
    virtual bool updateVector(size_t , const Position &, const Direction &, double) override { return false; }
    virtual bool updateVector(size_t, const Position &, const Vector3&) override { return false; }
    virtual bool setVectorColor(size_t , const ColorViz &) override { return false; }
    virtual bool setVectorsAspect(double, double, double) override { return false; }
};

class DummyFrameVisualization : public IFrameVisualization
{
public:

    virtual ~DummyFrameVisualization(){ };
    virtual size_t addFrame(const Transform&, double) override {return 0; };
    virtual bool setVisible(size_t , bool ) override {return false;};
    virtual size_t getNrOfFrames() const override {return 0; };
    virtual bool getFrameTransform(size_t , Transform& ) const override {return false;};
    virtual bool updateFrame(size_t, const Transform&) override {return false;};
};

/**
 * Dummy model visualization.
 */
class DummyModelVisualization : public IModelVisualization
{
    Model m_dummyModel;
    DummyJetsVisualization m_dummyJets;
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
    virtual bool setLinkColor(const LinkIndex &, const ColorViz &) { return false; }
    virtual bool resetLinkColor(const LinkIndex &) { return false; }
    virtual std::vector< std::string > getLinkNames() { return std::vector<std::string>(); };
    virtual bool setLinkVisibility(const std::string &, bool) { return false; }
    virtual std::vector<std::string> getFeatures() { return std::vector<std::string>(); }
    virtual bool setFeatureVisibility(const std::string& , bool) { return false; }
    virtual IJetsVisualization& jets() { return m_dummyJets;  }
    virtual Transform getWorldModelTransform() { return iDynTree::Transform::Identity(); }
    virtual Transform getWorldLinkTransform(const LinkIndex &) { return iDynTree::Transform::Identity(); }
};

class DummyTexturesHandler : public ITexturesHandler
{
public:

    virtual ~DummyTexturesHandler(){ };
    virtual ITexture* add(const std::string& , const VisualizerOptions&){return nullptr;};
    virtual ITexture* get(const std::string& ){return nullptr;};
};

}

#endif
