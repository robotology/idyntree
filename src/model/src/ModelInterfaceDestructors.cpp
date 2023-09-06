// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

/**
 * \file ModelInterfaceDestructors.cpp
 *
 * For a strange feature of C++, all interfaces (i.e. pure abstract classes)
 * require a destructor body for their destructor, even if it abstract.
 *
 * This file contains this "dummy" destructors for all the pure abstract classes of iDynTree Model.
 *
 */

#include <iDynTree/IJoint.h>

namespace iDynTree
{

IJoint::~IJoint()
{
}

}