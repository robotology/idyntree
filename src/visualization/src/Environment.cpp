/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "Environment.h"
#include "IrrlichtUtils.h"

namespace iDynTree
{

Environment::Environment(): m_sceneManager(0),
                            m_rootFrameNode(0),
                            m_gridLinesVisible(true)
{

}

Environment::~Environment()
{
    for(size_t i=0; i < m_lights.size(); i++)
    {
        m_lights[i]->removeLight();
        delete m_lights[i];
        m_lights[i] = 0;
    }

    m_lights.resize(0);
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
    if( elementKey == "floor_grid"  )
    {
        if( m_rootFrameNode )
        {
            m_rootFrameNode->setVisible(isVisible);

            retValue = true;
        }
    }

    if( elementKey == "root_frame" )
    {
        m_gridLinesVisible = isVisible;

        retValue = true;
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

    int lightIdx = m_lights.size();

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
            m_lights[i] = 0;
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
