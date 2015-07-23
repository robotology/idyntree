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

#include <kdl_codyco/generalizedjntmotions.hpp>

namespace KDL
{
namespace CoDyCo
{
    GeneralizedJntMotions::GeneralizedJntMotions():
        base_twist(), jnt()
    {
    }

    GeneralizedJntMotions::GeneralizedJntMotions(const int nrOfDOFs):
        base_twist(), jnt(nrOfDOFs)
    {
    }

    GeneralizedJntMotions::GeneralizedJntMotions(const KDL::Twist & _base_twist, const KDL::JntArray & _jnt):
        base_twist(_base_twist), jnt(_jnt)
    {
    }


    GeneralizedJntMotions::GeneralizedJntMotions(const GeneralizedJntMotions& arg):
        base_twist(arg.base_twist), jnt(arg.jnt)
    {
    }

    GeneralizedJntMotions& GeneralizedJntMotions::operator = (const GeneralizedJntMotions& arg)
    {
        base_twist = arg.base_twist;
        jnt = arg.jnt;
		return *this;
    }


    GeneralizedJntMotions::~GeneralizedJntMotions()
    {
    }

    void GeneralizedJntMotions::setNrOfDOFs(unsigned int nrOfDOFs)
    {
        jnt.resize(nrOfDOFs);
    }

    unsigned int GeneralizedJntMotions::getNrOfDOFs()
    {
        return jnt.rows();
    }


}

}
