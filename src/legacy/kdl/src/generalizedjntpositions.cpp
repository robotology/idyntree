// Copyright  (C)  2014  Silvio Traversaro <silvio dot traversaro at iit dot it>

/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <kdl_codyco/generalizedjntpositions.hpp>

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
		return *this;
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
