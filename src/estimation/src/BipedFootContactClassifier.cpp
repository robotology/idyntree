// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "iDynTree/BipedFootContactClassifier.h"

namespace iDynTree 
{

BipedFootContactClassifier::BipedFootContactClassifier(const SchmittParams& leftFootSchmittParams, 
                                             const SchmittParams& rightFootSchmittParams) : m_primaryFoot(RIGHT_FOOT),                                                                                          
                                                                                            m_leftFootContactState(true),
                                                                                            m_rightFootContactState(true),
                                                                                            m_pattern(ALTERNATE_CONTACT)
{
     m_leftFootContactClassifier = std::unique_ptr<ContactStateMachine>( new ContactStateMachine(leftFootSchmittParams));
     m_rightFootContactClassifier = std::unique_ptr<ContactStateMachine>( new ContactStateMachine(rightFootSchmittParams));
}


void BipedFootContactClassifier::updateFootContactState(double currentTime, double leftFootNormalForce, double rightFootNormalForce)
{
    m_leftFootContactClassifier.get()->contactMeasurementUpdate(currentTime, leftFootNormalForce);
    m_rightFootContactClassifier.get()->contactMeasurementUpdate(currentTime, rightFootNormalForce);
    m_leftFootContactState = m_leftFootContactClassifier.get()->contactState();
    m_rightFootContactState = m_rightFootContactClassifier.get()->contactState();
    
    detectFeetTransition();
}


void BipedFootContactClassifier::detectFeetTransition()
{
    ContactStateMachine::contactTransition leftFootTransition = m_leftFootContactClassifier.get()->contactTransitionMode();
    ContactStateMachine::contactTransition rightFootTransition = m_rightFootContactClassifier.get()->contactTransitionMode();
    
    switch (m_pattern)
    {
        case SwitchingPattern::ALTERNATE_CONTACT:
            if (m_primaryFoot == LEFT_FOOT)
            {                
                if ( rightFootTransition == ContactStateMachine::CONTACT_MAKE && m_leftFootContactState == true)
                {
                    m_primaryFoot = RIGHT_FOOT; 
                }
                else if (leftFootTransition == ContactStateMachine::STABLE_ONCONTACT || rightFootTransition == ContactStateMachine::STABLE_ONCONTACT)
                {
                    m_primaryFoot = LEFT_FOOT;
                }
                else if (leftFootTransition == ContactStateMachine::STABLE_OFFCONTACT)
                {
                    m_primaryFoot = RIGHT_FOOT;
                    if (rightFootTransition == ContactStateMachine::STABLE_OFFCONTACT)
                    {
                        m_primaryFoot = UNKNOWN_FOOT;
                    }
                }
            }
            else if (m_primaryFoot == RIGHT_FOOT)
            {                
                if ( leftFootTransition == ContactStateMachine::CONTACT_MAKE && m_rightFootContactState == true)
                {
                    m_primaryFoot = LEFT_FOOT;
                }
                
                else if (rightFootTransition == ContactStateMachine::STABLE_ONCONTACT || leftFootTransition == ContactStateMachine::STABLE_ONCONTACT)
                {
                    m_primaryFoot = RIGHT_FOOT;
                }
                
                else if (rightFootTransition == ContactStateMachine::STABLE_OFFCONTACT)
                {
                    m_primaryFoot = LEFT_FOOT;
                    if (leftFootTransition == ContactStateMachine::STABLE_OFFCONTACT)
                    {
                        m_primaryFoot = UNKNOWN_FOOT;
                    }
                }
            }
            else if (m_primaryFoot == UNKNOWN_FOOT)
            {
                // check if left foot has become active first, then check right foot
                if (leftFootTransition == ContactStateMachine::CONTACT_MAKE || leftFootTransition == ContactStateMachine::STABLE_ONCONTACT)
                {
                    m_primaryFoot = LEFT_FOOT;
                }
                else if (rightFootTransition == ContactStateMachine::CONTACT_MAKE || rightFootTransition == ContactStateMachine::STABLE_ONCONTACT)
                {
                    m_primaryFoot = RIGHT_FOOT;
                }
            }
            break;
            
        case SwitchingPattern::LATEST_ACTIVE_CONTACT:
            m_primaryFoot = LEFT_FOOT;
            break;
            
        case SwitchingPattern::DEFAULT_CONTACT:
            m_primaryFoot = LEFT_FOOT;
            break;
    }
}    

    
}

