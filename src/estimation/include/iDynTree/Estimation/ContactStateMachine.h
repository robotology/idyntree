/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro, Prashanth Ramadoss
 * email: silvio.traversaro@iit.it, prashanth.ramadoss@iit.it
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef IDYNTREE_CONTACTSTATEMACHINE_H
#define IDYNTREE_CONTACTSTATEMACHINE_H
#include <iostream>
#include <memory>
#include "SchmittTrigger.h"

namespace iDynTree
{
    /**
     * struct to hold schmitt trigger device parameters
     */
    struct SchmittParams
    {
        double stableTimeContactMake;
        double stableTimeContactBreak;
        double contactMakeForceThreshold;
        double contactBreakForceThreshold;
    };
    
   /**
    * Contact State Machine class for binary contact state detection
    * Contains a Schmitt Trigger device for updating the contact states
    * using the contact normal force acting on the contact link and
    * Determines contact transitions using simple binary switching logic
    * 
    * Can be used to determine stable contacts, contact breaking and contact making.
    * The parameters to the Schmitt Trigger are passed as a struct and is the Schmitt 
    * Trigger is instantiated in the class constructor. 
    * 
    * NOTE: There are no default parameters to the Schmitt Trigger. These parameters are set through 
    * the constructor during instantiation.
    * 
    * This class does not exactly abstract the Schmitt Trigger class. Schmitt Trigger methods are still accessible through 
    * the m_contactSchmitt object. This class uses a generic Schmitt Trigger object and augments its functionality
    * specific to physical contacts based scenarios.
    */
    class ContactStateMachine
    {
    public:
        /**
         * Enumeration of contact transitions
         */
        enum contactTransition
        {
            /**
             * previous state: off contact, current state: off contact
             */
            STABLE_OFFCONTACT, // 0
            
            /**
             * previous state: on contact, current state: on contact
             */
            STABLE_ONCONTACT,  // 1
            
            /**
             * previous state: on contact, current state: off contact
             */
            CONTACT_BREAK,     // 2
            
            /**
             * previous state: off contact, current state: on contact
             */
            CONTACT_MAKE,       // 3

 	    /**
	     * Unknown transition
             */
            UNKNOWN_TRANSITION = -1
        };
        
        /**
         * Constructor
         * @param s const reference to a struct containing schmitt trigger device parameters
         */
        ContactStateMachine(const SchmittParams& s);
                
        /**
         * Calls schmitt trigger device update 
         * @param currentTime time
         * @param contactNormalForce normal force acting on the contact link in consideration
         */
        void contactMeasurementUpdate(double currentTime, double contactNormalForce);
        
        /**
         * Calls schmitt trigger device reset 
         */
        void resetDevice() { m_contactSchmitt.get()->resetDevice(); }
        
        /**
         * Get current contact state
         * @return true, if in contact, false otherwise
         */
        bool contactState() { return m_currentState; }
        
        /**
         * Determines contact transitions using simple binary switching logic
         * @return contactTransition enumerated value
         */
        contactTransition contactTransitionMode();
        
        /**
         * Get time of last contact state update
         * @return time
         */
        double lastUpdateTime();
        
        /**
         * unique pointer to the schmitt trigger device
         */
        std::unique_ptr<SchmittTrigger> m_contactSchmitt;
    private:
        // previous contact state
        bool m_previousState;
        
        // current contact state
        bool m_currentState;
        
        // transtion mode based on previous and current contact states
        int m_tranisitionMode;
        
    };
}
#endif
