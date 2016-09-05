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

Environment::Environment(): m_rootFrameNode(0),
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

}
