/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

/**
 * \file ModelInterfaceDestructors.cpp
 *
 * For a strange feature of C++, all interfaces (i.e. pure abstract classes)
 * require a destructor body for their destructor, even if it abstract.
 *
 * This file contains this "dummy" destructors for all the pure abstract classes of iDynTree Model.
 *
 */

#include <iDynTree/Model/IJoint.h>

namespace iDynTree
{

IJoint::~IJoint()
{
}

}