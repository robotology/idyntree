/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_ENVIRONMENT_H
#define IDYNTREE_ENVIRONMENT_H

#include <iDynTree/Visualizer.h>

#include <irrlicht.h>

namespace iDynTree
{

class Environment : public IEnvironment
{
public:
    irr::scene::ISceneNode * m_rootFrameNode;
    bool m_gridLinesVisible;

    Environment();
    virtual ~Environment();

    virtual std::vector< std::string > getElements();
    virtual bool setElementVisibility(const std::string elementKey, bool isVisible);

};

}

#endif
