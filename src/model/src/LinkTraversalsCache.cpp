/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
