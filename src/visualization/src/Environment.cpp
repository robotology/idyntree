/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

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
}

Environment::~Environment()
{
    close();
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
    if( elementKey == "root_frame"  )
    {
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
    m_lights[lightIdx]->addLight(lightName,m_sceneManager->addLightSceneNode());

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
            std::vector<Light*>::iterator it;
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



}
