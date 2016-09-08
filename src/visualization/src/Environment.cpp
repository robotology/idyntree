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
    this->m_backgroundColor = idyntree2irrllicht(backgroundColor);
}

void Environment::setAmbientLight(const ColorViz& ambientLight)
{
    this->m_sceneManager->setAmbientLight(idyntree2irrllicht(ambientLight));
}

std::vector< std::string > Environment::getLights()
{
    std::vector< std::string > lightsNames;

    for(size_t i=0; i < m_lights.size(); i++)
    {
        lightsNames.push_back(m_lights[i].name);
    }

    return lightsNames;
}

bool Environment::addLight(const std::string& lightName)
{
    for(size_t i=0; i < m_lights.size(); i++)
    {
        if( m_lights[i].name == lightName )
        {
            reportError("Environment","addLight","Light with the requested name already exists, impossible to add it.");
        }
    }

    // Add a new light
    pimpl->m_sceneManager->addLightSceneNode();
}

ILight& Environment::lightViz(const std::string& lightName)
{

}

bool Environment::removeLight(const std::__cxx11::string& lightName)
{
}



}
