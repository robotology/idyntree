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

#include <kdl_codyco/generalizedjnttorques.hpp>

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
		return *this;
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
