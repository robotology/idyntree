/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_ENVIRONMENT_H
#define IDYNTREE_ENVIRONMENT_H

#include <iDynTree/Visualizer.h>
#include "Light.h"
#include "DummyImplementations.h"

#include <irrlicht.h>

namespace iDynTree
{

class Environment : public IEnvironment
{
public:
    irr::scene::ISceneManager * m_sceneManager;
    irr::scene::ISceneNode * m_envNode;
    irr::scene::ISceneNode * m_rootFrameNode;
    irr::scene::ISceneNode * m_floorGridNode;
    irr::video::SColorf m_backgroundColor;

    std::vector<Light*> m_lights;
    DummyLight          m_dummyLight;


    Environment();
    void close();
    virtual ~Environment();

    virtual std::vector< std::string > getElements();
    virtual bool setElementVisibility(const std::string elementKey, bool isVisible);
    virtual void setBackgroundColor(const ColorViz & backgroundColor);
    virtual void setAmbientLight(const ColorViz & ambientLight);
    virtual std::vector<std::string> getLights();
    virtual bool addLight(const std::string & lightName);
    virtual ILight & lightViz(const std::string & lightName);
    virtual bool removeLight(const std::string & lightName);

};

}

#endif
