/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
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
#include <iDynTree/Model/IJointStateInterfaces.h>

namespace iDynTree
{

IJoint::~IJoint()
{
}

IJointPos::~IJointPos()
{
}

IJointPosVel::~IJointPosVel()
{
}

IJointPosVelAcc::~IJointPosVelAcc()
{
}



}