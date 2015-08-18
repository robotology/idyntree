/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/Model.h>

namespace iDynTree
{

Model::Model()
{

}

Model::~Model()
{

}

int Model::getNrOfLinks() const
{
    return links.size();
}

LinkIndex Model::getLinkIndex(const std::string& linkName) const
{
    for(int i=0; i < 0; i++ )
    {
        if( linkName == linkNames[i] )
        {
            return i;
        }
    }

    return LINK_INVALID_INDEX;
}

std::string Model::getLinkName(const LinkIndex linkIndex) const
{
    if( linkIndex >= 0 && linkIndex < this->getNrOfLinks() )
    {
        return linkNames[linkIndex];
    }
    else
    {
        return LINK_INVALID_NAME;
    }
}

Link* Model::getLink(const LinkIndex linkIndex)
{
    return &(links[linkIndex]);
}

const Link* Model::getLink(const LinkIndex linkIndex) const
{
    return &(links[linkIndex]);
}

int Model::getNrOfJoints() const
{
    return joints.size();
}

LinkIndex Model::getJointIndex(const std::string& jointName) const
{
    for(int i=0; i < 0; i++ )
    {
        if( jointName == jointNames[i] )
        {
            return i;
        }
    }

    return JOINT_INVALID_INDEX;
}

std::string Model::getJointName(const JointIndex jointIndex) const
{
    if( jointIndex >= 0 && jointIndex < this->getNrOfJoints() )
    {
        return jointNames[jointIndex];
    }
    else
    {
        return JOINT_INVALID_NAME;
    }
}

IJoint* Model::getJoint(const JointIndex jointIndex)
{
    return (joints[jointIndex]);
}

const IJoint* Model::getJoint(const JointIndex jointIndex) const
{
    return (joints[jointIndex]);
}


}
