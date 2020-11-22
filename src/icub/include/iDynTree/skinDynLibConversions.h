/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_SKINDYNLIB_CONVERSIONS_H
#define IDYNTREE_SKINDYNLIB_CONVERSIONS_H

#include <map>
#include <string>

#include <iDynTree/Model/Indices.h>
#include <iDynTree/Estimation/ExternalWrenchesEstimation.h>

#include <iCub/skinDynLib/dynContactList.h>
#include <iCub/skinDynLib/skinContactList.h>

namespace iDynTree
{
class Model;
class LinkContactWrenches;

/**
 * Identifier for a link and a body frame in skinDynLib.
 *
 * \ingroup iDynTreeICUB
 */
class skinDynLibLinkID
{
public:
    int body_part;
    int local_link_index;

    bool operator<(const skinDynLibLinkID& k) const
    {
        if(this->body_part < k.body_part)
        {
           return true;
        }
        else if(this->body_part > k.body_part)
        {
           return false;
        }
        else
        {
            return this->local_link_index < k.local_link_index;
        }
    }

    bool operator==(const skinDynLibLinkID& k) const
    {
        return (this->body_part == k.body_part &&
                this->local_link_index == k.local_link_index);
    }
};

/**
 * Identifier for a link and frame couple in an iDynTree model.
 *
 * \ingroup iDynTreeICUB
 */
class iDynTreeLinkAndFrame
{
     public:
          LinkIndex  link_index;
          FrameIndex frame_index;
};


/**
 * \brief Helper for conversion between iDynTree data structures and skinDynLib data structures.
 *
 * There are several differences to handle:
 *  * In iDynTree, link and frames and identified by name
 *    and (in the context of an instantiated model) by their index, while in
 *    skinDynLib link are identified by the bodyPart, a numeric id representing a part of the robot
 *    the linkIndex, a progressive numeric id uniquely identifyng  the link in the part.
 *  * The link frame of a link in iDynTree does not match the link frame assumed in skinDynLib.
 *    For this reason we need to specify an additional frame to specify the frame of the link
 *    used by skinDynLib, to properly convert the contact data back and forth.
 *
 * \ingroup iDynTreeICUB
 */
class skinDynLibConversionsHelper
{
private:
    std::map<skinDynLibLinkID,iDynTreeLinkAndFrame> skinDynLibLinkMap;

public:
    /**
     * This function enables interoperability between the iDynTree library
     * and the iCub skinDynLib library.
     *
     * This function can be used to assign a match between a skinDynLib link,
     * and a iDynTree link.
     *
     * In skinDynLib a link is represented by two numbers:
     *   * the bodyPart, a numeric id representing a part of the robot
     *   * the linkIndex, a progressive numeric id uniquely identifyng  the link in the part
     * We associate this to two iDynTree concept
     *   * the link_name of the considered link
     *   * the frame_name of the frame used by skinDynLib when dealing with that link.
     *         This frame can be the link frame, in this case frame_name == link_name,
     *          otherwise another frame can be used, under the constraint that this frame
     *          must be rigidly attached to the considered link.
     *
     */
    bool addSkinDynLibAlias(const Model& model,
                            const std::string iDynTree_link_name, const std::string iDynTree_frame_name,
                            const int skinDynLib_body_part, const int skinDynLib_link_index);

    /**
     * Retrieve the skinDynLib alias of a link, added to the class using the addSkinDynLibAlias method.
     */
    bool getSkinDynLibAlias(const Model & model,
                            const std::string iDynTree_link_name,
                                  std::string & iDynTree_frame_name,
                                  int & skinDynLib_body_part,
                                  int & skinDynLib_link_index) const;

    /**
     * Retrieve the skinDynLib alias of a link, added to the class using the addSkinDynLibAlias method.
     */
    bool getSkinDynLibAlias(const Model & model,
                            const LinkIndex iDynTree_link_index,
                                  FrameIndex & iDynTree_frame_index,
                             int & skinDynLib_body_part,
                             int & skinDynLib_link_index) const;

    /**
     * Convert a skinDynLib identifier to a iDynTree link/frame identifier.
     */
    bool skinDynLib2iDynTree(const int skinDynLib_body_part,
                             const int skinDynLib_link_index,
                             LinkIndex & iDynTree_link_index,
                             FrameIndex & iDynTree_frame_index) const;
    /**
     * Remove a alias in the form (body_part, link_index) for a link
     */
    bool removeSkinDynLibAlias(const Model & model, const std::string linkName);

    /**
     * Convert a dynContactList to a LinkUnknownWrenchContacts.
     *
     * The contactId contained in the dynContactList is preserved and saved
     * in the appropriate attribute in the LinkUnknownWrenchContacts class.
     */
    bool fromSkinDynLibToiDynTree(const Model& model,
                                  const iCub::skinDynLib::dynContactList & dynList,
                                        LinkUnknownWrenchContacts & unknowns);

    /**
     * Convert a skinContactList to a LinkUnknownWrenchContacts.
     *
     * The contactId contained in the skinContactList is preserved and saved
     * in the appropriate attribute in the LinkUnknownWrenchContacts class.
     */
    bool fromSkinDynLibToiDynTree(const Model& model,
                                  const iCub::skinDynLib::skinContactList & skinList,
                                        LinkUnknownWrenchContacts & unknowns);

    /**
     * Convert a LinkContactWrenches to a iCub::skinDynLib::dynContactList.
     *
     * This function creates a new dynContactList.
     *
     */
    bool fromiDynTreeToSkinDynLib(const Model& model,
                                  const LinkContactWrenches & contactWrenches,
                                        iCub::skinDynLib::dynContactList & dynList);

    /**
     * Update an existing skinContactList in which some forces and torque are
     * unknown using the estimated contact wrenches contained in
     * a LinkContactWrenches. The matching between the skinContactList and
     * the LinkContactWrenches contacts is done through the contactId, that then
     * should be consistent between the two functions.
     */
    bool updateSkinContactListFromLinkContactWrenches(const Model& model,
                                                      const LinkContactWrenches & contactWrenches,
                                                            iCub::skinDynLib::skinContactList & skinContactListToUpdate);
};

}

#endif
