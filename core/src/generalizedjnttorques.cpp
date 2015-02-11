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

#include "generalizedjnttorques.hpp"

namespace KDL
{
namespace CoDyCo
{
    GeneralizedJntTorques::GeneralizedJntTorques():
        base_wrench(), jnt()
    {
    }

    GeneralizedJntTorques::GeneralizedJntTorques(const int nrOfDOFs):
        base_wrench(), jnt(nrOfDOFs)
    {
    }

    GeneralizedJntTorques::GeneralizedJntTorques(const KDL::Wrench _base_wrench, const KDL::JntArray _jnt):
        base_wrench(_base_wrench), jnt(_jnt)
    {
    }


    GeneralizedJntTorques::GeneralizedJntTorques(const GeneralizedJntTorques& arg):
        base_wrench(arg.base_wrench), jnt(arg.jnt)
    {
    }

    GeneralizedJntTorques& GeneralizedJntTorques::operator = (const GeneralizedJntTorques& arg)
    {
        base_wrench = arg.base_wrench;
        jnt = arg.jnt;
    }


    GeneralizedJntTorques::~GeneralizedJntTorques()
    {
    }

    void GeneralizedJntTorques::setNrOfDOFs(unsigned int nrOfDOFs)
    {
        jnt.resize(nrOfDOFs);
    }

    unsigned int GeneralizedJntTorques::getNrOfDOFs()
    {
        return jnt.rows();
    }


}

}
