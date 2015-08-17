/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/Link.h>

namespace iDynTree
{

Link::Link()
{

}

Link::~Link()
{

}

void Link::setInertia(SpatialInertia& _inertia)
{
    this->m_inertia = _inertia;
}

const SpatialInertia& Link::getInertia() const
{
    return this->m_inertia;
}



}
