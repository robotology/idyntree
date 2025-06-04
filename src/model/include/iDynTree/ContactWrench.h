// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_CONTACT_WRENCH_H
#define IDYNTREE_CONTACT_WRENCH_H

#include <iDynTree/Position.h>
#include <iDynTree/Wrench.h>

#include <iDynTree/Indices.h>
#include <iDynTree/LinkState.h>


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

        /**
         * Unique id identifing the contact.
         * This id is propagated to the contact wrench data structure.
         * It is implemented mainly for compatibility with the skinDynLib library.
         */
        std::size_t m_contactId;

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

        std::size_t& contactId();

        const Position & contactPoint() const;
        const Wrench   & contactWrench() const;
        const std::size_t& contactId() const;

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
        LinkContactWrenches(std::size_t nrOfLinks = 0);
        LinkContactWrenches(const Model & model);

        /**
         *
         *
         * @param[in] nrOfLinks the number of links to which resize this object
         */
        void resize(std::size_t nrOfLinks);
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
        * Add a new contact for a link.
        *
        * @param[in] linkIndex the index of the link for which the contact is added.
        * @param[in] newContact the new contact wrench to add.
        */
        void addNewContactForLink(const LinkIndex linkIndex, const ContactWrench& newContact);

        /**
        * Add a new contact for a frame.
        * If the specified frame is not a link frame, the method automatically convert the
        * wrench to the relative link frame.
        *
        * @param[in] model the model class for getting frame information.
        * @param[in] frameIndex the index of the frame in which you are expressing the new wrench.
        * @param[in] newContact the new wrench to add.
        * @return true if all went well, false otherwise
        */
        bool addNewContactInFrame(const Model & model,
                                const FrameIndex frameIndex,
                                const ContactWrench& newContact);

        /**
         * Get a human readable description of the LinkWrenchContacts (for debug)
         */
        std::string toString(const Model& model) const;
    };


}


#endif /* IDYNTREE_CONTACT_WRENCH_H */
