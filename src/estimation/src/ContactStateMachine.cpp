#include "iDynTree/Estimation/ContactStateMachine.h"
namespace iDynTree
{

ContactStateMachine::ContactStateMachine(const SchmittParams& s) : m_previousState(true), 
                                                                   m_currentState(true)                                                                   
{
    m_contactSchmitt = std::unique_ptr<SchmittTrigger>(new SchmittTrigger(s.stableTimeContactBreak, 
                                                                          s.stableTimeContactMake,
                                                                          s.contactBreakForceThreshold,
                                                                          s.contactMakeForceThreshold));
    m_contactSchmitt.get()->setInitialState(m_previousState);
}


ContactStateMachine::contactTransition ContactStateMachine::contactTransitionMode()
{
    if (m_previousState == 0 && m_currentState == 0)
        return STABLE_OFFCONTACT; // 0
    
    if (m_previousState == 0 && m_currentState == 1)
        return CONTACT_MAKE;      // 3
    
    if (m_previousState == 1 && m_currentState == 0)
        return CONTACT_BREAK;     // 2
    
    if (m_previousState == 1 && m_currentState == 1)
        return STABLE_ONCONTACT;  // 1

    return UNKNOWN_TRANSITION;
}

double ContactStateMachine::lastUpdateTime()
{
    return m_contactSchmitt.get()->getElapsedTime();
}

void ContactStateMachine::contactMeasurementUpdate(double currentTime, double contactNormalForce)
{
    m_contactSchmitt.get()->updateDevice(currentTime, contactNormalForce);
    m_previousState = m_currentState;
    m_currentState = m_contactSchmitt.get()->getState();
}
    
}

