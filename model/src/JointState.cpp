/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/JointState.h>

namespace iDynTree
{
    unsigned int NullJointPos::getNrOfPosCoords() const
    {
        return 0;
    }

    const IRawVector& NullJointPos::pos() const
    {
        return this->m_pos;
    }

    IRawVector& NullJointPos::pos()
    {
        return this->m_pos;
    }

    NullJointPos::~NullJointPos()
    {

    }
}
