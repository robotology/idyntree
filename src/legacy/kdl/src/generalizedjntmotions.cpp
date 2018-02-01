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
