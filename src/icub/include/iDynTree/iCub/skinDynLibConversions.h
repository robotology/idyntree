/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SKINDYNLIB_CONVERSIONS_H
#define IDYNTREE_SKINDYNLIB_CONVERSIONS_H

#include <string>

#include <iDynTree/Model/Indeces.h>
#include <iDynTree/Estimation/ExternalWrenchesEstimation.h>

namespace iDynTree
{

class skinDynLibConversionsHelper
{
private:
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
    bool addSkinDynLibAlias(const std::string iDynTree_link_name, const std::string iDynTree_frame_name,
                            const int skinDynLib_body_part, const int skinDynLib_link_index);

    /**
     * Retrieve the skinDynLib alias of a link, added to the class using the addSkinDynLibAlias method.
     */
    bool getSkinDynLibAlias(const std::string iDynTree_link_name,
                                  std::string & iDynTree_frame_name,
                                  int & skinDynLib_body_part,
                                  int & skinDynLib_link_index) ;

    /**
     * Retrieve the skinDynLib alias of a link, added to the class using the addSkinDynLibAlias method.
     */
    bool getSkinDynLibAlias(const LinkIndex iDynTree_link_index,
                                  LinkIndex & iDynTree_frame_index,
                             int & skinDynLib_body_part,
                             int & skinDynLib_link_index);

    /**
     * Convert a skinDynLib identifier to a iDynTree link/frame identifier.
     */
    bool skinDynLib2iDynTree(const int skinDynLib_body_part,
                             const int skinDynLib_link_index,
                             int & iDynTree_link_index,
                             int & iDynTree_frame_index);
    /**
     * Remove a alias in the form (body_part, link_index) for a link
     */
    bool removeSkinDynLibAlias(const std::string linkName);
}

#endif
