// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "Environment.h"
#include "IrrlichtUtils.h"

namespace iDynTree
{

Environment::Environment(): m_sceneManager(nullptr),
                            m_rootFrameNode(nullptr),
                            m_floorGridNode(nullptr)
{

}

void Environment::close()
{
    if (m_floorGridNode){
        m_floorGridNode->drop();
        m_floorGridNode = nullptr;
    }

    for(size_t i=0; i < m_lights.size(); i++)
    {
        m_lights[i]->removeLight();
        delete m_lights[i];
        m_lights[i] = nullptr;
    }

    m_lights.resize(0);

    if (m_sceneManager)
    {
        m_sceneManager->drop();
        m_sceneManager = nullptr;
    }
}

Environment::~Environment()
{
    close();
}

void Environment::init(irr::scene::ISceneManager *sceneManager, double rootFrameArrowsDimension)
{
    m_sceneManager = sceneManager;
    m_sceneManager->grab();
    m_envNode       = m_sceneManager->addEmptySceneNode();
    m_rootFrameNode = addFrameAxes(m_sceneManager,m_envNode, rootFrameArrowsDimension);
    m_floorGridNode = addFloorGridNode(m_sceneManager,m_envNode);
    m_backgroundColor = irr::video::SColorf(0.0,0.4,0.4,1.0);

    // Add default light (sun, directional light pointing backwards
    addVizLights(m_sceneManager);
    std::string sunName = "sun";
    addLight(sunName);
    ILight & sun = lightViz(sunName);
    sun.setType(DIRECTIONAL_LIGHT);
    sun.setDirection(iDynTree::Direction(0,0,-1));
    sun.setDiffuseColor(iDynTree::ColorViz(0.7,0.7,0.7,1.0));
    sun.setSpecularColor(iDynTree::ColorViz(0.1,0.1,0.1,1.0));
    sun.setAmbientColor(iDynTree::ColorViz(0.1,0.1,0.1,1.0));
}

std::vector< std::string > Environment::getElements()
{
    // If you modify this function, remember to modify also
    // the documentation in IEnvironment
    std::vector<std::string> elements;
    elements.push_back("floor_grid");
    elements.push_back("root_frame");

    return elements;
}

bool Environment::setElementVisibility(const std::string elementKey, bool isVisible)
{
    bool retValue = false;
    if((elementKey == "world_frame") || (elementKey == "root_frame")) //"root_frame is kept for retrocompatibility due to a previous misalignement with the docs
    {
        if (elementKey == "root_frame")
        {
            reportWarning("Environment", "setElementVisibility", "\"root_frame\" is deprecated. Use \"world_frame\" instead.");
        }

        if( m_rootFrameNode )
        {
            m_rootFrameNode->setVisible(isVisible);

            retValue = true;
        }
    }

    if( elementKey == "floor_grid" )
    {
        if( m_floorGridNode )
        {
            m_floorGridNode->setVisible(isVisible);

            retValue = true;
        }
    }

    return retValue;
}

void Environment::setBackgroundColor(const ColorViz& backgroundColor)
{
    this->m_backgroundColor = idyntree2irrlicht(backgroundColor);
}

void Environment::setAmbientLight(const ColorViz& ambientLight)
{
    this->m_sceneManager->setAmbientLight(idyntree2irrlicht(ambientLight));
}

std::vector< std::string > Environment::getLights()
{
    std::vector< std::string > lightsNames;

    for(size_t i=0; i < m_lights.size(); i++)
    {
        lightsNames.push_back(m_lights[i]->getName());
    }

    return lightsNames;
}

bool Environment::addLight(const std::string& lightName)
{
    for(size_t i=0; i < m_lights.size(); i++)
    {
        if( m_lights[i]->getName() == lightName )
        {
            reportError("Environment","addLight","Light with the requested name already exists, impossible to add it.");
            return false;
        }
    }

    size_t lightIdx = m_lights.size();

    // Add a new light
    m_lights.push_back(new Light());
    m_lights[lightIdx]->addLight(lightName,m_sceneManager->addLightSceneNode(m_envNode));

    return true;
}

ILight& Environment::lightViz(const std::string& lightName)
{
    for(size_t i=0; i < m_lights.size(); i++)
    {
        if( m_lights[i]->getName() == lightName )
        {
            return *m_lights[i];
        }
    }

    std::stringstream ss;
    ss << "Light name " << lightName << " does not exist.";
    reportError("Environment","addLight",ss.str().c_str());
    return m_dummyLight;
}

bool Environment::removeLight(const std::string& lightName)
{
    for(size_t i=0; i < m_lights.size(); i++)
    {
        if( m_lights[i]->getName() == lightName )
        {
            m_lights[i]->removeLight();
            delete m_lights[i];
            m_lights[i] = nullptr;
            std::vector<Light*>::iterator it = m_lights.begin();
            std::advance(it,i);
            m_lights.erase(it);
            return true;
        }
    }

    std::stringstream ss;
    ss << "Light name " << lightName << " does not exist.";
    reportError("Environment","removeLight",ss.str().c_str());
    return false;
}

void Environment::setFloorGridColor(const ColorViz &floorGridColor)
{
    if (m_floorGridNode)
    {
        m_floorGridNode->setGridColor(idyntree2irrlicht(floorGridColor).toSColor());
    }
}



}
