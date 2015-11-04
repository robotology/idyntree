/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/Model.h>

#include <cassert>

namespace iDynTree
{

FreeFloatingPos::FreeFloatingPos(const Model& model)
{
    resize(model);
}

IJointPos * allocateJointPosOfSize(const unsigned int nrOfPosCoords)
{
    IJointPos * retPtr;

    assert(nrOfPosCoords <= 6);

    switch(nrOfPosCoords)
    {
        case 0:
            retPtr = new NullJointPos;
            break;
        case 1:
            retPtr = new JointPos<1>();
            break;
        case 2:
            retPtr = new JointPos<2>();
            break;
        case 3:
            retPtr = new JointPos<3>();
            break;
        case 4:
            retPtr = new JointPos<4>();
            break;
        case 5:
            retPtr = new JointPos<5>();
            break;
        case 6:
            retPtr = new JointPos<6>();
            break;
        default:
            assert(false);
            break;
    }

    return retPtr;
}

void FreeFloatingPos::clearJointPos()
{
    for(size_t jnt = 0; jnt < this->m_jointPos.size(); jnt++)
    {
        if( this->m_jointPos[jnt] )
        {
            delete this->m_jointPos[jnt];
            this->m_jointPos[jnt] = 0;
        }
    }

    this->m_jointPos.resize(0);
}


void FreeFloatingPos::buildJointPos(const Model& model)
{
    this->clearJointPos();

    this->m_jointPos.resize(model.getNrOfJoints());

    for(size_t jnt = 0; jnt < this->m_jointPos.size(); jnt++)
    {
        unsigned int jointNrOfPosCoords = model.getJoint(jnt)->getNrOfPosCoords();
        this->m_jointPos[jnt] = allocateJointPosOfSize(jointNrOfPosCoords);
    }

    return;
}

void FreeFloatingPos::resize(const Model& model)
{
    nrOfPosCoords = model.getNrOfPosCoords();

    if( model.getNrOfJoints() != this->m_jointPos.size() )
    {
        this->buildJointPos(model);
    }

    for(unsigned int jnt=0; jnt < model.getNrOfJoints(); jnt++)
    {
        if( model.getJoint(jnt)->getNrOfPosCoords() != this->m_jointPos[jnt]->getNrOfPosCoords() )
        {
            if( this->m_jointPos[jnt] )
            {
                delete this->m_jointPos[jnt];
            }
            this->m_jointPos[jnt] = allocateJointPosOfSize(model.getJoint(jnt)->getNrOfPosCoords());
        }
    }
}


Transform& FreeFloatingPos::worldBasePos()
{
    return this->m_worldBasePos;
}

IJointPos& FreeFloatingPos::jointPos(const JointIndex jointIndex)
{
    assert(jointIndex >= 0 && jointIndex <= this->m_jointPos.size());
    return *(this->m_jointPos[jointIndex]);
}

const Transform& FreeFloatingPos::worldBasePos() const
{
    return this->m_worldBasePos;
}

const IJointPos& FreeFloatingPos::jointPos(const JointIndex jointIndex) const
{
    assert(jointIndex >= 0 && jointIndex <= this->m_jointPos.size());
    return *(this->m_jointPos[jointIndex]);
}

unsigned int FreeFloatingPos::getNrOfPosCoords() const
{
    return nrOfPosCoords;
}


FreeFloatingPos::~FreeFloatingPos()
{
    this->clearJointPos();
}

}
