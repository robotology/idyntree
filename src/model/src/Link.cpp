/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Model/Link.h>

namespace iDynTree
{

Link::Link(): m_index(LINK_INVALID_INDEX)
{

}

SpatialInertia& Link::inertia()
{
    return m_inertia;
}

const SpatialInertia& Link::inertia() const
{
    return m_inertia;
}



void Link::setInertia(SpatialInertia& _inertia)
{
    this->m_inertia = _inertia;
}

const SpatialInertia& Link::getInertia() const
{
    return this->m_inertia;
}

void Link::setIndex(LinkIndex& _index)
{
    this->m_index = _index;
}

LinkIndex Link::getIndex() const
{
    return this->m_index;
}



}
