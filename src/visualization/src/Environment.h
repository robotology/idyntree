// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_ENVIRONMENT_H
#define IDYNTREE_ENVIRONMENT_H

#include <iDynTree/Visualizer.h>
#include "Light.h"
#include "DummyImplementations.h"
#include "FloorGridSceneNode.h"

#include <irrlicht.h>

namespace iDynTree
{

class Environment : public IEnvironment
{
public:
    irr::scene::ISceneManager * m_sceneManager;
    irr::scene::ISceneNode * m_envNode;
    irr::scene::ISceneNode * m_rootFrameNode;
    CFloorGridSceneNode * m_floorGridNode;
    irr::video::SColorf m_backgroundColor;

    std::vector<Light*> m_lights;
    DummyLight          m_dummyLight;


    Environment();
    void close();
    virtual ~Environment();

    void init(irr::scene::ISceneManager* sceneManager, double rootFrameArrowsDimension);

    virtual std::vector< std::string > getElements();
    virtual bool setElementVisibility(const std::string elementKey, bool isVisible);
    virtual void setBackgroundColor(const ColorViz & backgroundColor);
    virtual void setAmbientLight(const ColorViz & ambientLight);
    virtual std::vector<std::string> getLights();
    virtual bool addLight(const std::string & lightName);
    virtual ILight & lightViz(const std::string & lightName);
    virtual bool removeLight(const std::string & lightName);
    virtual void setFloorGridColor(const ColorViz & floorGridColor);

};

}

#endif
