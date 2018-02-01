/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <kdl/tree.hpp>
#include <sstream>
#include <algorithm>
#include <map>
#include <stack>
#include <iostream>
#include <cassert>

#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_codyco/undirectedtree.hpp>
#include <kdl_codyco/utils.hpp>

namespace KDL {
namespace CoDyCo {

    int Traversal::getNrOfVisitedLinks() const
    {
        return order.size();
    }

    LinkMap::const_iterator Traversal::getOrderedLink(int order_number) const
    {
        return order[order_number];
    }

    LinkMap::const_iterator Traversal::getBaseLink() const
    {
        assert(order.size() != 0);
        return order[0];
    }


    LinkMap::const_iterator Traversal::getParentLink(int link_index) const
    {
        return parent[link_index];
    }

    LinkMap::const_iterator Traversal::getParentLink(LinkMap::const_iterator link_iterator) const
    {
        return parent[link_iterator->getLinkIndex()];
    }


}
}//end of namespace
