// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
#ifndef IDYNTREE_BIPED_FOOT_CONTACT_CLASSIFIER_H
#define IDYNTREE_BIPED_FOOT_CONTACT_CLASSIFIER_H

#include <memory>
#include "ContactStateMachine.h"


namespace iDynTree
{
    /**
     * Enumeration of switching pattern
     */
    enum SwitchingPattern
    {
        /**
         * Switching active foot between left and right at every double stance 
         */
        ALTERNATE_CONTACT,
        
        /**
         * Setting active foot to the one that made contact most recently  
         * @warning this pattern remains unimplemented in the current version of this class
         *          using this pattern will always return left foot as the active foot
         */
        LATEST_ACTIVE_CONTACT,
        
        /**
         * Fixing active foot to the default foot defined by the user  
         * @warning this pattern remains unimplemented in the current version of this class
         *          using this pattern will always return left foot as the active foot
         */
        DEFAULT_CONTACT
    };

   /* Foot Contact Classifier class for determining the primary foot in contact
    * with the ground surface. Contains contact state machines for each foot: left and right,
    * and implements the Schmitt Trigger thresholding on each foot to determine the primary
    * foot which is in contact according to a switching pattern criteria. 
    * The switching pattern is primarily used to change the frames of reference for the Legged Odometry (LO) 
    * while one of the two foot breaks contact with the ground surface, since LO assumes that atleast one
    * foot is in contact with ground at any instant of time. However, it is also used to assign 
    * the primary foot in case of double stance.
    * 
    * In the current version of this class, switching logic is implemented only for 
    * ALTERNATE_CONTACT patterns vastly considered during walking tasks. This class is aimed
    * to be extended towards other switching patterns like LATEST_ACTIVE_CONTACT, DEFAULT_CONTACT
    * for implementation along side different task based controllers. 
    */    
    class BipedFootContactClassifier
    {
    public:
       /**
        * Enumeration of foot in contact
        */
        enum contactFoot
        {
            LEFT_FOOT,   // 0
            RIGHT_FOOT,  // 1
            UNKNOWN_FOOT // 2
        };
        
        /**
         * Constructor
         * @params leftFootSchmittParams const ref to struct containing parameters for the left foot schmitt trigger device
         * @params rightFootSchmittParams const ref to struct containing parameters for the right foot schmitt trigger device
         */
        BipedFootContactClassifier(const SchmittParams& leftFootSchmittParams, const SchmittParams& rightFootSchmittParams);
       
        /**
         * Updates the contact state machine for both the foot, determines the current state
         * and detects foot transition for setting the primary foot(active foot)
         * @param currentTime time
         * @param leftNormalForce z-component of the force acting on the left foot 
         * @param rightNormalForce z-component of the force acting on the right foot 
         */
        void updateFootContactState(double currentTime, double leftFootNormalForce, double rightFootNormalForce);
        
        /**
         * Get the primary foot
         * @return contactFoot left, right or unknown
         */
        contactFoot getPrimaryFoot() { return m_primaryFoot; }
        
        /**
         * Get left foot contact state
         * @return true if in contact, false otherwise
         */
        bool getLeftFootContactState() { return m_leftFootContactState; }
        
        /**
         * Get right foot contact state
         * @return true if in contact, false otherwise
         */
        bool getRightFootContactState() { return m_rightFootContactState; }
        
        /**
         * set switching pattern to be considered for determining primary foot
         * @warning no default value is considered, remember to call this method after instantiation
         * @param pattern switching pattern
         */
        void setContactSwitchingPattern(SwitchingPattern pattern) { m_pattern = pattern; } 
        
        /**
         * set  primary foot
         * This method was mainly intended to be called by an external process 
         * to set the primary foot in the initial setting, before any contact is broken. 
         * In case it is set to UNKNOWN_FOOT, it waits for the foot normal force measurements (checks left first and then right), 
         * and activates corresponding foot, handled in the detectTransitions() method.
         * @param foot primary foot
         */
        void setPrimaryFoot(contactFoot foot) { m_primaryFoot = foot; }
       
        // unique pointer to contact state machine for left foot
        std::unique_ptr<ContactStateMachine> m_leftFootContactClassifier;
        
        // unique pointer to contact state machine for right foot
        std::unique_ptr<ContactStateMachine> m_rightFootContactClassifier;
    
    private:        
        /**
         * Determine the primary foot depending on the switching pattern
         * compares the contact transition modes on each foot and 
         * sets the primary foot accordingly
         */
        void detectFeetTransition();
       
        // active/primary foot
        contactFoot m_primaryFoot;
        
        // left foot contact state
        bool m_leftFootContactState;
        
        // right foot contact state
        bool m_rightFootContactState;

        // switching pattern
        SwitchingPattern m_pattern;
    };
}
#endif