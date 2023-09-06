// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include <iDynTree/Model.h>
#include <iDynTree/ContactWrench.h>

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

std::size_t& ContactWrench::contactId()
{
    return m_contactId;
}


const std::size_t& ContactWrench::contactId() const
{
    return m_contactId;
}


LinkContactWrenches::LinkContactWrenches(std::size_t nrOfLinks)
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

void LinkContactWrenches::resize(std::size_t nrOfLinks)
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

size_t LinkContactWrenches::getNrOfLinks() const
{
    return m_linkContactWrenches.size();
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

bool LinkContactWrenches::computeNetWrenches(LinkNetExternalWrenches& netWrenches) const
{
    // Resize the output if necessary
    size_t nrOfLinks =  netWrenches.getNrOfLinks();
    if( netWrenches.getNrOfLinks() != m_linkContactWrenches.size() )
    {
        netWrenches.resize(m_linkContactWrenches.size());
    }

    for(LinkIndex l=0; l < static_cast<LinkIndex>(nrOfLinks); l++)
    {
        netWrenches(l).zero();

        // sum all the contact wrenches
        size_t nrOfContacts = this->getNrOfContactsForLink(l);
        for(size_t c=0; c < nrOfContacts; c++)
        {
            // Each contact wrench is expressed with respect to the contact point
            // and with the orientation of the link frame, so we need to translate it
            // to the link frame
            const ContactWrench & contact = this->contactWrench(l,c);

            Transform link_H_contact(Rotation::Identity(),contact.contactPoint());

            Wrench link_wrench_due_to_contact = link_H_contact*contact.contactWrench();

            netWrenches(l) = netWrenches(l) + link_wrench_due_to_contact;
        }
    }

    return true;
}

std::string LinkContactWrenches::toString(const Model& model) const
{
    std::stringstream ss;

    size_t nrOfLinks = m_linkContactWrenches.size();
    for(size_t l=0; l < nrOfLinks; l++)
    {
        size_t nrOfContacts = this->getNrOfContactsForLink(l);

        if( nrOfContacts > 0 )
        {
            ss << "Contact wrenches on link " << model.getLinkName(l) << ":" << std::endl;
            for(size_t c=0; c < nrOfContacts; c++ )
            {
                ss << "Wrench contact with pos: " << this->contactWrench(l,c).contactPoint().toString() << ","
                                          "wrench: " << this->contactWrench(l,c).contactWrench().toString() << std::endl;
            }
        }
    }
    return ss.str();
}

}






