/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_CONTACT_WRENCH_H
#define IDYNTREE_CONTACT_WRENCH_H

#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Wrench.h>

#include <iDynTree/Model/Indeces.h>
#include <iDynTree/Model/LinkState.h>


#include <vector>

namespace iDynTree
{
    class Model;

    /**
     * \brief A wrench excerted on a link due to an external contact.
     *
     * The contact wrench is represented by two quantities the contactPoint
     * and the contactWrench .
     */
    class ContactWrench
    {
    private:
        Position m_contactPoint;
        Wrench   m_contactWrench;

    public:
        /**
         * \brief Position of a point on the contact surface.
         *
         * This is the position of a point (expressed in the link frame) that lies
         * on the contact surface.
         *
         * Its precise definition is application dependent.
         *
         * If the ContactWrench class is used to represent a pure force
         * applied on a point, the contactPoint is the point in which the
         * pure force is applied.
         *
         * If the ContactWrench class is used to represent contact information
         * coming from a tacticle system, the contact point may be the average
         * between all the taxels (tactile elements) belonging to the contact.
         */
        Position & contactPoint();

        /**
         * \brief Wrench that an external element excert on a link through the contact.
         *
         * This wrench (expressed with respect to the contactPoint and with the orientation
         * of the link frame) is the wrench excerted on the link by an external element.
         *
         */
        Wrench   & contactWrench();
        const Position & contactPoint() const;
        const Wrench   & contactWrench() const;
    };

    /**
     * A set of ContactWrench for each link, representing all the contacts
     * between the model and the external environment.
     *
     */
    class LinkContactWrenches
    {
    private:
        std::vector< std::vector<ContactWrench> > m_linkContactWrenches;

    public:
        /**
         * Create a LinkWrenches vector, with the size given
         * by nrOfLinks .
         *
         * @param[in] nrOfLinks the size of the vector.
         */
        LinkContactWrenches(unsigned int nrOfLinks = 0);
        LinkContactWrenches(const Model & model);

        /**
         *
         *
         * @param[in]
         */
        void resize(unsigned int nrOfLinks);
        void resize(const Model & model);

        /**
         * Get the number of external contacts for a given link.
         */
        size_t getNrOfContactsForLink(const LinkIndex linkIndex) const;

        /**
         * Set the number of external contacts for a given link.
         */
        void setNrOfContactsForLink(const LinkIndex linkIndex, const size_t nrOfContacts);

        /**
         * Get the number of links.
         */
        size_t getNrOfLinks() const;

        /**
         * Get a specific ContactWrench
         *
         * @param[in] linkIndex the index of the link for which the contact is retrieved
         * @param[in] contactIndex a index (between 0 and getNrOfContactsForLink(link)-1 ) identifing the specific contact.
         */
        ContactWrench& contactWrench(const LinkIndex linkIndex, const size_t contactIndex);

        const ContactWrench& contactWrench(const LinkIndex linkIndex, const size_t contactIndex) const;

        /**
         *
         * \brief Compute the net wrenches vector from the contacts wrenches vector.
         *
         * This is just a loop that sums all the contact wrenches for every link
         * and store the results (expressed in the link frame) in the netWrenches vector.
         *
         * @return true if all went well, false otherwise.
         */
        bool computeNetWrenches(LinkNetExternalWrenches & netWrenches) const;


        /**
         * Get a human readable description of the LinkUnknownWrenchContacts (for debug)
         */
        std::string toString(const Model& model) const;
    };


}


#endif /* IDYNTREE_CONTACT_WRENCH_H */