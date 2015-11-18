// Copyright  (C)  2013 Silvio Traversaro <silvio dot traversaro at iit dot it>
// Author: Silvio Traversaro

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

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
