/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Visualizer.h>

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
 * Dummy environment.
 */
class DummyEnvironment : public IEnvironment
{
public:
    virtual ~DummyEnvironment() {};
    virtual bool setElementVisibility(const std::string /*elementKey*/, bool /*isVisible*/) {return false;}
    virtual std::vector< std::string > getElements() {  return std::vector< std::string >(); }
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

}
