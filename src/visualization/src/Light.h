/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_LIGHT_H
#define IDYNTREE_LIGHT_H

#include <iDynTree/Visualizer.h>

#include <irrlicht.h>

namespace iDynTree
{

class Light : public ILight
{
    irr::scene::ILightSceneNode * m_lightSceneNode;
    std::string m_name;
public:
    Light();
    virtual ~Light();

    void addLight(const std::string name,
                  irr::scene::ILightSceneNode * lightSceneNode);
    void removeLight();

    virtual const std::string & getName() const;
    virtual void setType(const LightType type);
    virtual LightType getType();
    virtual void setPosition(const iDynTree::Position & cameraPos);
    virtual iDynTree::Position getPosition();
    virtual void setDirection(const Direction& lightDirection);
    virtual Direction getDirection();
    virtual void setAmbientColor(const ColorViz & ambientColor);
    virtual ColorViz getAmbientColor();
    virtual void setSpecularColor(const ColorViz & ambientColor);
    virtual ColorViz getSpecularColor();
    virtual void setDiffuseColor(const ColorViz & ambientColor);
    virtual ColorViz getDiffuseColor();
};

}

#endif
