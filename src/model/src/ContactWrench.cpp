/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/ContactWrench.h>

namespace iDynTree
{

Position& ContactWrench::contactPoint()
{
    return m_contactPoint;
}

const Position& ContactWrench::contactPoint() const
{
    return m_contactPoint;
}

Wrench& ContactWrench::contactWrench()
{
    return m_contactWrench;
}


const Wrench& ContactWrench::contactWrench() const
{
    return m_contactWrench;
}

LinkContactWrenches::LinkContactWrenches(unsigned int nrOfLinks)
{
    this->resize(nrOfLinks);
}


LinkContactWrenches::LinkContactWrenches(const iDynTree::Model& model)
{
    this->resize(model);
}

void LinkContactWrenches::resize(const iDynTree::Model& model)
{
    resize(model.getNrOfLinks());
}

void LinkContactWrenches::resize(unsigned int nrOfLinks)
{
    // To avoid dynamic memory allocation at runtime
    // we make sure that the vector have at least spaces for 3 contacts
    const size_t reservedSlots = 3;
    m_linkContactWrenches.resize(nrOfLinks);
    for(size_t l=0; l < nrOfLinks; l++)
    {
        m_linkContactWrenches[l].resize(0);
        m_linkContactWrenches[l].reserve(reservedSlots);
    }
}

const ContactWrench& LinkContactWrenches::contactWrench(const LinkIndex linkIndex, const size_t contactIndex) const
{
    return m_linkContactWrenches[linkIndex][contactIndex];
}

ContactWrench& LinkContactWrenches::contactWrench(const LinkIndex linkIndex, const size_t contactIndex)
{
    return m_linkContactWrenches[linkIndex][contactIndex];
}

size_t LinkContactWrenches::getNrOfContactsForLink(const LinkIndex linkIndex) const
{
    return m_linkContactWrenches[linkIndex].size();
}

void LinkContactWrenches::setNrOfContactsForLink(const LinkIndex linkIndex, const size_t nrOfContacts)
{
    m_linkContactWrenches[linkIndex].resize(nrOfContacts);
    return;
}







}






