// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_SKINDYNLIB_CONVERSIONS_IMPLEMENTATION_H
#define IDYNTREE_SKINDYNLIB_CONVERSIONS_IMPLEMENTATION_H

#include <iDynTree/skinDynLibConversions.h>

#include <iDynTree/Position.h>
#include <iDynTree/Wrench.h>

#include <iDynTree/Model.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/ContactWrench.h>

#include <iDynTree/YARPConversions.h>

#include <cassert>

namespace iDynTree
{

inline bool skinDynLibConversionsHelper::addSkinDynLibAlias(const Model& model, const std::string iDynTree_link_name, const std::string iDynTree_frame_name,
                                                            const int skinDynLib_body_part, const int skinDynLib_link_index)
{
   int iDynTree_link_index = model.getLinkIndex(iDynTree_link_name);
   if( iDynTree_link_index < 0 )
   {
       std::cerr << "[ERROR] addSkinDynLibAlias : link " << iDynTree_link_name << " not found in the model " << std::endl;
   }

   int iDynTree_frame_index = model.getFrameIndex(iDynTree_frame_name);
   if( iDynTree_frame_index < 0 )
   {
       std::cerr << "[ERROR] addSkinDynLibAlias : frame " << iDynTree_frame_name << " not found in the model " << std::endl;
   }

   skinDynLibLinkID sdl_id;
   sdl_id.body_part = skinDynLib_body_part;
   sdl_id.local_link_index = skinDynLib_link_index;

   iDynTreeLinkAndFrame idyntree_id;
   idyntree_id.link_index = iDynTree_link_index;
   idyntree_id.frame_index = iDynTree_frame_index;

   //Remove any existing alias for this link to avoid anomalies
   this->removeSkinDynLibAlias(model,iDynTree_link_name);

   skinDynLibLinkMap.insert(std::pair<skinDynLibLinkID,iDynTreeLinkAndFrame>(sdl_id,idyntree_id));

   return true;
}

inline bool skinDynLibConversionsHelper::getSkinDynLibAlias(const Model& model,
                                                            const std::string iDynTree_link_name,
                                                            std::string & iDynTree_frame_name,
                                                            int & skinDynLib_body_part,
                                                            int & skinDynLib_link_index) const
{
   int iDynTree_link_index = model.getLinkIndex(iDynTree_link_name);
   if( iDynTree_link_index < 0 )
   {
       std::cerr << "[ERROR] getSkinDynLibAlias : link " << iDynTree_link_name << " not found " << std::endl;
       return false;
   }

   for(std::map<skinDynLibLinkID,iDynTreeLinkAndFrame>::const_iterator it = skinDynLibLinkMap.begin();
       it != skinDynLibLinkMap.end(); it++ )
   {
       if( it->second.link_index == iDynTree_link_index )
       {
           skinDynLib_body_part = it->first.body_part;
           skinDynLib_link_index = it->first.local_link_index;
           int iDynTree_frame_index = it->second.frame_index;
           iDynTree_frame_name = model.getFrameName(iDynTree_frame_index);
           break;
       }
   }

   return true;

}

inline bool skinDynLibConversionsHelper::getSkinDynLibAlias(const Model& model,
                                                            const LinkIndex iDynTree_link_index, FrameIndex & iDynTree_frame_index,
                                                            int & skinDynLib_body_part, int & skinDynLib_link_index) const
{
  if( iDynTree_link_index < 0 || iDynTree_link_index >= static_cast<LinkIndex>(model.getNrOfLinks()) ) return false;

  // TODO \todo What is this crazyness??? Linear search of a map??
   for(std::map<skinDynLibLinkID,iDynTreeLinkAndFrame>::const_iterator it = skinDynLibLinkMap.begin();
       it != skinDynLibLinkMap.end(); it++ )
   {
       if( it->second.link_index == iDynTree_link_index )
       {
           skinDynLib_body_part = it->first.body_part;
           skinDynLib_link_index = it->first.local_link_index;
           iDynTree_frame_index = it->second.frame_index;
           return true;
       }
   }

   return false;

}

inline bool skinDynLibConversionsHelper::removeSkinDynLibAlias(const Model& model, std::string link)
{
   int link_index = model.getLinkIndex(link);
   if( link_index < 0 )
   {
       std::cerr << "[ERROR] removeSkinDynLibAlias : link " << link << " not found " << std::endl;
   }

   for(std::map<skinDynLibLinkID,iDynTreeLinkAndFrame>::iterator it = skinDynLibLinkMap.begin();
       it != skinDynLibLinkMap.end(); it++ )
   {
       if( it->second.link_index == link_index )
       {
           skinDynLibLinkMap.erase(it);
           break;
       }
   }

   return true;
}

inline bool skinDynLibConversionsHelper::skinDynLib2iDynTree(const int skinDynLib_body_part,
                                                             const int skinDynLib_link_index,
                                                             LinkIndex & iDynTree_link_index,
                                                             FrameIndex & iDynTree_frame_index) const
{
    skinDynLibLinkID skinID;
    skinID.body_part = skinDynLib_body_part;
    skinID.local_link_index = skinDynLib_link_index;

    ::std::map<skinDynLibLinkID,iDynTreeLinkAndFrame>::const_iterator it = skinDynLibLinkMap.find(skinID);

    if( it == skinDynLibLinkMap.end() )
    {
        std::cerr << "[ERROR] DynTree::skinDynLib2iDynTree : skinDynLib link "
                  << skinDynLib_body_part << " " << skinDynLib_link_index << " not found " << std::endl;
        return false;
    }

    iDynTree_link_index  = it->second.link_index;
    iDynTree_frame_index = it->second.frame_index;

    return true;
}

/**
 * Templated version of fromSkinDynLibToiDynTree, useful to implement
 * the exact same function for dynContactList and skinContactList, that
 * have exactly the same interface, but they don't have any common ancestor
 * in the class structure.
 */
template<class contactsList>
bool fromSkinDynLibToiDynTreeHelper(const Model& model,
                                    const contactsList& dynList,
                                          LinkUnknownWrenchContacts& unknowns,
                                          const skinDynLibConversionsHelper& conversionHelper)
{
    unknowns.resize(model);

    // Reset the contact vectors
    for(size_t l=0; l < model.getNrOfLinks(); l++)
    {
        unknowns.setNrOfContactsForLink(l,0);
    }

    typename contactsList::const_iterator it;
    for(it = dynList.begin(); it!=dynList.end(); it++)
    {
        // Unknown
        UnknownWrenchContact unknownWrench;

        //get link index
        int skinDynLib_body_part = it->getBodyPart();
        int skinDynLib_link_index = it->getLinkNumber();

        LinkIndex iDynTree_link_index = LINK_INVALID_INDEX;
        FrameIndex iDynTree_skinFrame_index = FRAME_INVALID_INDEX;

        bool skinDynLib_ID_found = conversionHelper.skinDynLib2iDynTree(skinDynLib_body_part,skinDynLib_link_index,
                                                                        iDynTree_link_index,iDynTree_skinFrame_index);

        if( !skinDynLib_ID_found )
        {
            std::cerr << "[ERROR] skinDynLibConversionsHelper::fromSkinDynLibToiDynTree skinDynLib_ID_found not found, skipping contact" << std::endl;
            continue;
        }

        // Get the transform between the skinDynLib frame and the iDynTree link frame
        iDynTree::Transform link_H_skinDynLibFrame = model.getFrameTransform(iDynTree_skinFrame_index);

        Position skinFrame_contactPoint;
        // Copy the contact point in the unknown
        toiDynTree(it->getCoP(),skinFrame_contactPoint);
        unknownWrench.contactPoint = link_H_skinDynLibFrame*skinFrame_contactPoint;

        if(it->isForceDirectionKnown())
        {
            //1 UNKNOWN
            unknownWrench.unknownType = PURE_FORCE_WITH_KNOWN_DIRECTION;
            Direction skinDynLibDirection;
            toiDynTree(it->getForceDirection(),skinDynLibDirection);

            // The direction of the force will be expressed in the skinDynLib orientation
            // we need to convert it in the link frame orientation
            unknownWrench.forceDirection = link_H_skinDynLibFrame*skinDynLibDirection;
        }
        else
        {
            if(it->isMomentKnown())
            {
                unknownWrench.unknownType = PURE_FORCE;
            }
            else
            {
                //6 UNKNOWN
                unknownWrench.unknownType = FULL_WRENCH;
            }
        }

        // Save the contact ID
        unknownWrench.contactId = it->getId();

        unknowns.addNewContactForLink((LinkIndex)iDynTree_link_index,unknownWrench);
    }

    return true;
}
 
inline bool skinDynLibConversionsHelper::fromSkinDynLibToiDynTree(const Model& model,
                                                                  const iCub::skinDynLib::dynContactList& dynList,
                                                                  LinkUnknownWrenchContacts& unknowns)
{
    return fromSkinDynLibToiDynTreeHelper<iCub::skinDynLib::dynContactList>(model,dynList,unknowns,*this);
}

inline bool skinDynLibConversionsHelper::fromSkinDynLibToiDynTree(const Model& model,
                                                                  const iCub::skinDynLib::skinContactList& skinList,
                                                                  LinkUnknownWrenchContacts& unknowns)
{
    return fromSkinDynLibToiDynTreeHelper<iCub::skinDynLib::skinContactList>(model,skinList,unknowns,*this);
}



inline bool skinDynLibConversionsHelper::fromiDynTreeToSkinDynLib(const Model & model,
                                                                  const LinkContactWrenches& contactWrenches,
                                                                  iCub::skinDynLib::dynContactList& dynList)
{
    dynList.resize(0);
    yarp::sig::Vector forceCache(3), momentCache(3), positionCache(3);

    size_t nrOfLinks = model.getNrOfLinks();
    for(LinkIndex l=0; l < static_cast<LinkIndex>(nrOfLinks); l++)
    {
        size_t nrOfContacts = contactWrenches.getNrOfContactsForLink(l);

        if( nrOfContacts > 0 )
        {
            //get link index
            int skinDynLib_body_part = -1;
            int skinDynLib_link_index = -1;

            FrameIndex iDynTree_skinFrame_index = FRAME_INVALID_INDEX;

            bool skinDynLib_ID_found = getSkinDynLibAlias(model,l,iDynTree_skinFrame_index,skinDynLib_body_part,skinDynLib_link_index);

            if( !skinDynLib_ID_found )
            {
                std::cerr << "[ERROR] skinDynLibConversionsHelper::fromSkinDynLibToiDynTree skinDynLib_ID_found not found, skipping contact" << std::endl;
                continue;
            }

            // Get the transform between the skinDynLib frame and the iDynTree link frame
            iDynTree::Transform skinDynLibFrame_H_link = model.getFrameTransform(iDynTree_skinFrame_index).inverse();

            for(size_t c=0; c < nrOfContacts; c++)
            {
                // We need to convert the position of the contact from the iDynTree frame to the skinDynLib frame
                Position skinDynLib_contactPoint = skinDynLibFrame_H_link*(contactWrenches.contactWrench(l,c).contactPoint());

                // The wrench is always expressed wrt to the contact point, so we need just to change the orientation
                Wrench skinDynLib_wrench = skinDynLibFrame_H_link.getRotation()*(contactWrenches.contactWrench(l,c).contactWrench());

                toYarp(skinDynLib_wrench.getLinearVec3(),forceCache);
                toYarp(skinDynLib_wrench.getAngularVec3(),momentCache);
                toYarp(skinDynLib_contactPoint,positionCache);

                iCub::skinDynLib::dynContact skinLibContact;
                skinLibContact.setCoP(positionCache);
                skinLibContact.setForceMoment(forceCache,momentCache);
                skinLibContact.setBodyPart((iCub::skinDynLib::BodyPart)skinDynLib_body_part);
                skinLibContact.setLinkNumber(skinDynLib_link_index);

                dynList.push_back(skinLibContact);
            }
        }
    }

    return true;
}

inline bool skinDynLibConversionsHelper::updateSkinContactListFromLinkContactWrenches(const Model& model,
                                                                                      const LinkContactWrenches& contactWrenches,
                                                                                      iCub::skinDynLib::skinContactList& skinContactListToUpdate)
{
    yarp::sig::Vector wrenchCache(6), positionCache(3);

    size_t nrOfLinks = model.getNrOfLinks();
    for(LinkIndex l=0; l < static_cast<LinkIndex>(nrOfLinks); l++)
    {
        size_t nrOfContacts = contactWrenches.getNrOfContactsForLink(l);

        if( nrOfContacts > 0 )
        {
            //get link index
            int skinDynLib_body_part = -1;
            int skinDynLib_link_index = -1;

            FrameIndex iDynTree_skinFrame_index = FRAME_INVALID_INDEX;

            bool skinDynLib_ID_found = getSkinDynLibAlias(model,l,iDynTree_skinFrame_index,skinDynLib_body_part,skinDynLib_link_index);

            if( !skinDynLib_ID_found )
            {
                std::cerr << "[ERROR] skinDynLibConversionsHelper::fromSkinDynLibToiDynTree : link " << model.getLinkName(l) <<  " not found, skipping contact" << std::endl;
                continue;
            }

            // Get the transform between the skinDynLib frame and the iDynTree link frame
            iDynTree::Transform skinDynLibFrame_H_link = model.getFrameTransform(iDynTree_skinFrame_index).inverse();

            for(size_t c=0; c < nrOfContacts; c++)
            {
                unsigned long contactId = contactWrenches.contactWrench(l,c).contactId();

                // We need to convert the position of the contact from the iDynTree frame to the skinDynLib frame
                Position skinDynLib_contactPoint = skinDynLibFrame_H_link*(contactWrenches.contactWrench(l,c).contactPoint());

                // The wrench is always expressed wrt to the contact point, so we need just to change the orientation
                Wrench skinDynLib_wrench = skinDynLibFrame_H_link.getRotation()*(contactWrenches.contactWrench(l,c).contactWrench());

                // To set the contact in the skinContactList, we need to search it using the contactId
                bool contactFound = false;
                for(iCub::skinDynLib::skinContactList::iterator it = skinContactListToUpdate.begin();
                    it != skinContactListToUpdate.end(); it++)
                {
                    if( it->getId() == contactId )
                    {
                        contactFound = true;

                        toYarp(skinDynLib_wrench,wrenchCache);

                        it->setForceMoment(wrenchCache);

                        break;
                    }
                }

                if(!contactFound)
                {
                    toYarp(skinDynLib_wrench,wrenchCache);
                    toYarp(skinDynLib_contactPoint,positionCache);

                    iCub::skinDynLib::skinContact skinLibContact;
                    skinLibContact.setCoP(positionCache);
                    skinLibContact.setForceMoment(wrenchCache);
                    skinLibContact.setBodyPart((iCub::skinDynLib::BodyPart)skinDynLib_body_part);
                    skinLibContact.setLinkNumber(skinDynLib_link_index);

                    skinContactListToUpdate.push_back(skinLibContact);
                }
            }
        }
    }

    return true;
}



}

#endif
