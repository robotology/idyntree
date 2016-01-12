/*
 * Copyright (C) 2016 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 */

#include <iDynTree/iCub/skinDynLibConversions.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/ContactWrench.h>

#include <cassert>

namespace iDynTree
{

bool skinDynLibConversionsHelper::addSkinDynLibAlias(const Model& model, const std::string iDynTree_link_name, const std::string iDynTree_frame_name,
                                                     const int skinDynLib_body_part, const int skinDynLib_link_index)
{
   int iDynTree_link_index = model.getLinkIndex(iDynTree_link_name);
   if( iDynTree_link_index < 0 )
   {
       std::cerr << "[ERR] addSkinDynLibAlias : link " << iDynTree_link_name << " not found in the model " << std::endl;
   }

   int iDynTree_frame_index = model.getFrameIndex(iDynTree_frame_name);
   if( iDynTree_frame_index < 0 )
   {
       std::cerr << "[ERR] addSkinDynLibAlias : frame " << iDynTree_frame_name << " not found in the model " << std::endl;
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

bool skinDynLibConversionsHelper::getSkinDynLibAlias(const Model& model,
                                                     const std::string iDynTree_link_name,
                                                     std::string & iDynTree_frame_name,
                                                     int & skinDynLib_body_part,
                                                     int & skinDynLib_link_index)
{
   int iDynTree_link_index = model.getLinkIndex(iDynTree_link_name);
   if( iDynTree_link_index < 0 )
   {
       std::cerr << "[ERR] getSkinDynLibAlias : link " << iDynTree_link_name << " not found " << std::endl;
       return false;
   }

   for(std::map<skinDynLibLinkID,iDynTreeLinkAndFrame>::iterator it = skinDynLibLinkMap.begin();
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

bool skinDynLibConversionsHelper::getSkinDynLibAlias(const Model& model,
                                                     const int iDynTree_link_index, int & iDynTree_frame_index,
                                                     int & skinDynLib_body_part, int & skinDynLib_link_index)
{
  if( iDynTree_link_index < 0 || iDynTree_link_index >= model.getNrOfLinks() ) return false;

  // TODO \todo What is this crazyness??? Linear search of a map??
   for(std::map<skinDynLibLinkID,iDynTreeLinkAndFrame>::iterator it = skinDynLibLinkMap.begin();
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

bool skinDynLibConversionsHelper::removeSkinDynLibAlias(const Model& model, std::string link)
{
   int link_index = model.getLinkIndex(link);
   if( link_index < 0 )
   {
       std::cerr << "[ERR] removeSkinDynLibAlias : link " << link << " not found " << std::endl;
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

bool skinDynLibConversionsHelper::skinDynLib2iDynTree(const int skinDynLib_body_part,
                                                      const int skinDynLib_link_index,
                                                      int & iDynTree_link_index,
                                                       int & iDynTree_frame_index)
{
    skinDynLibLinkID skinID;
    skinID.body_part = skinDynLib_body_part;
    skinID.local_link_index = skinDynLib_link_index;

    ::std::map<skinDynLibLinkID,iDynTreeLinkAndFrame>::iterator it = skinDynLibLinkMap.find(skinID);

    if( it == skinDynLibLinkMap.end() )
    {
        std::cerr << "[ERR] DynTree::skinDynLib2iDynTree : skinDynLib link "
                  << skinDynLib_body_part << " " << skinDynLib_link_index << " not found " << std::endl;
        return false;
    }

    iDynTree_link_index  = it->second.link_index;
    iDynTree_frame_index = it->second.frame_index;

    return true;
}

bool skinDynLibConversionsHelper::fromSkinDynLibToiDynTree(const Model& model,
                                                           const iCub::skinDynLib::dynContactList& dynList,
                                                           LinkUnknownWrenchContacts& unknowns)
{
    unknowns.resize(model);

    iCub::skinDynLib::dynContactList::const_iterator it;
    for(it = dynList.begin(); it!=dynList.end(); it++)
    {
        // Unknown
        UnknownWrenchContact unknownWrench;

        //get link index
        int skinDynLib_body_part = it->getBodyPart();
        int skinDynLib_link_index = it->getLinkNumber();

        int iDynTree_link_index = -1;
        int iDynTree_skinFrame_index = -1;

        bool skinDynLib_ID_found = skinDynLib2iDynTree(skinDynLib_body_part,skinDynLib_link_index,
                                                       iDynTree_link_index,iDynTree_skinFrame_index);

        if( !skinDynLib_ID_found )
        {
            std::cerr << "[ERR] skinDynLibConversionsHelper::fromSkinDynLibToiDynTree skinDynLib_ID_found not found, skipping contact" << std::endl;
            continue;
        }

        // Get the transform between the skinDynLib frame and the iDynTree link frame
        iDynTree::Transform link_H_skinFrame = model.getFrameTransform(iDynTree_skinFrame_index);

        Position skinFrame_contactPoint;
        // Copy the contact point in the unknown
        assert(false);
        //iDyn(it->getCoP(),skinFrame_contactPoint);
        unknownWrench.contactPoint = link_H_skinFrame*skinFrame_contactPoint;


        if(it->isForceDirectionKnown())
        {
            //1 UNKNOWN
            unknownWrench.unknownType = PURE_FORCE_WITH_KNOWN_DIRECTION;
            assert(false);
            //fromYarp(it->(),unknownWrench.forceDirection);
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
    }

}


bool skinDynLibConversionsHelper::fromiDynTreeToSkinDynLib(const Model & model,
                                                           const LinkContactWrenches& contactWrenches,
                                                                 iCub::skinDynLib::dynContactList& dynList)
{
    return true;
}


}