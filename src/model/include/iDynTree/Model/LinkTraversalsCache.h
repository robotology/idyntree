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

#ifndef IDYNTREE_MODEL_LINKTRAVERSALSCACHE_H
#define IDYNTREE_MODEL_LINKTRAVERSALSCACHE_H

#include <vector>
#include <iDynTree/Model/Indeces.h>

namespace iDynTree {
    class Traversal;
    class Model;
    
    class LinkTraversalsCache;
}

/**
 * Link traversal cache, store a traversal for each link in the model.
 *
 * Class that stores a traversal for each link in the model.
 * It actually computes the traversal on the first time that a given traversal
 * is requested
 */
class iDynTree::LinkTraversalsCache
{
private:
    std::vector<iDynTree::Traversal *> m_linkTraversals;
    void deleteTraversals();

public:
    /**
     * Default constructur
     */
    LinkTraversalsCache();

    /**
     * Default destructor
     */
    ~LinkTraversalsCache();

    
    /**
     * Resize the cache to contains the specified number of Traversals
     *
     * @param nrOfLinks number of traversal
     */
    void resize(unsigned int nrOfLinks);


    /**
     * Resize the cache to contains the number of Traversals for the specified model
     *
     * @param model the model
     */
    void resize(const iDynTree::Model& model);


    /**
     * return the traversal for the specified link in the specified model
     *
     * If the traversal has been cached this function returns the cached traversal
     * otherwise it creates the new traversal, caches it and returns it, thus
     * performin a memory allocation.
     *
     * @param model the model
     * @param linkIdx the index to be considered as base
     * @return the traversal for the specified model link
     */
    Traversal& getTraversalWithLinkAsBase(const iDynTree::Model & model,
                                          const iDynTree::LinkIndex linkIdx);
};


#endif /* end of include guard: IDYNTREE_MODEL_LINKTRAVERSALSCACHE_H */
