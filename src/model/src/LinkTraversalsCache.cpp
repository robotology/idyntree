/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * email: silvio.traversaro@iit.it
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "iDynTree/Model/LinkTraversalsCache.h"

#include "iDynTree/Model/Traversal.h"
#include "iDynTree/Model/Model.h"

#include <cassert>

namespace iDynTree {

LinkTraversalsCache::LinkTraversalsCache()
{

}

LinkTraversalsCache::~LinkTraversalsCache()
{
    deleteTraversals();
}

void LinkTraversalsCache::deleteTraversals()
{
    for(size_t link=0; link < m_linkTraversals.size(); link++)
    {
        delete m_linkTraversals[link];
    }
    m_linkTraversals.resize(0);
}


void LinkTraversalsCache::resize(const Model& model)
{
    this->resize(model.getNrOfLinks());
}

void LinkTraversalsCache::resize(unsigned int nrOfLinks)
{
    deleteTraversals();
    m_linkTraversals.resize(nrOfLinks);
    for(size_t link=0; link < m_linkTraversals.size(); link++)
    {
        m_linkTraversals[link] = new Traversal();
    }
}

Traversal& LinkTraversalsCache::getTraversalWithLinkAsBase(const Model & model, const LinkIndex linkIdx)
{
    assert(model.isValidLinkIndex(linkIdx));
    
    if( m_linkTraversals[linkIdx]->getNrOfVisitedLinks() == 0 )
    {
        model.computeFullTreeTraversal(*m_linkTraversals[linkIdx],linkIdx);
    }

    return *m_linkTraversals[linkIdx];
}
}
