// Copyright  (C)  2014  Silvio Traversaro <silvio dot traversaro at iit dot it>

// Version: 1.0
// Author: Silvio Traversaro <silvio dot traversaro at iit dot it>
// Maintainer: Silvio Traversaro <silvio dot traversaro at iit dot it>

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

#include "kdl_codyco/generalizedjntpositions.hpp"

namespace KDL
{
namespace CoDyCo
{
    GeneralizedJntPositions::GeneralizedJntPositions():
        base_pos(), jnt_pos()
    {
    }

    GeneralizedJntPositions::GeneralizedJntPositions(const int nrOfDOFs):
        base_pos(), jnt_pos(nrOfDOFs)
    {
    }

    GeneralizedJntPositions::GeneralizedJntPositions(const KDL::Frame & _base_pos, const KDL::JntArray & _jnt_pos):
        base_pos(_base_pos), jnt_pos(_jnt_pos)
    {
    }


    GeneralizedJntPositions::GeneralizedJntPositions(const GeneralizedJntPositions& arg):
        base_pos(arg.base_pos), jnt_pos(arg.jnt_pos)
    {
    }

    GeneralizedJntPositions& GeneralizedJntPositions::operator = (const GeneralizedJntPositions& arg)
    {
        base_pos = arg.base_pos;
        jnt_pos = arg.jnt_pos;
    }


    GeneralizedJntPositions::~GeneralizedJntPositions()
    {
    }

    void GeneralizedJntPositions::setNrOfDOFs(unsigned int nrOfDOFs)
    {
        jnt_pos.resize(nrOfDOFs);
    }

    unsigned int GeneralizedJntPositions::getNrOfDOFs()
    {
        return jnt_pos.rows();
    }


}

}
