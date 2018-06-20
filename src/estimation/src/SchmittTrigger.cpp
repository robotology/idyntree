#include "iDynTree/Estimation/SchmittTrigger.h"

namespace iDynTree 
{

SchmittTrigger::SchmittTrigger(double stableOFFTime, double stableONTime, double lowValueThreshold, double highValueThreshold)
{
    configure(stableOFFTime, stableONTime, lowValueThreshold, highValueThreshold);
    resetDevice();
}


void SchmittTrigger::resetDevice()
{
    m_timer = 0;
    m_currentState = true;
    m_rawValue = 0.;
    m_previousTime = -1;
    m_verbose = 0;
}


void SchmittTrigger::configure(double stableOFFTime, double stableONTime, double lowValueThreshold, double highValueThreshold)
{
    setStableOFFTime(stableOFFTime);
    setStableONTime(stableONTime);
    setHighValueThreshold(highValueThreshold);
    setLowValueThreshold(lowValueThreshold);
}

void SchmittTrigger::updateDevice(double currentTime, double rawValue)
{
    if (m_previousTime < 0)
    {
        if (currentTime > 0)
        {
            m_previousTime = 0;
        }
        else
        {
            m_previousTime = currentTime;
        }   
    }
    if (m_verbose) std::cout << "SchmittTrigger: Time:: " << currentTime << std::endl;
    m_rawValue = rawValue;
    
    if (m_verbose) std::cout << "SchmittTrigger: Value:: " << m_rawValue << std::endl;
    if (m_verbose) std::cout << "SchmittTrigger: Timer:: " << m_timer << std::endl;
    
    if (m_currentState == false)
    {   
        // Check for transition - if valid over a timeframe, then switch
        if (m_rawValue >= m_highValueThreshold)
        {               
            if (m_timer > m_stableONTime)
            {
                // rise to high
                m_currentState = true;
                if (m_verbose) std::cout << "SchmittTrigger: Raising high "  << std::endl;
            }
            else
            {
                // wait for timer
                m_timer += (currentTime - m_previousTime);
                if (m_verbose) std::cout << "SchmittTrigger: I'm low and waiting "  << std::endl;
            }            
        }
        else
        {
            // stable low - reset timer
            m_timer = 0;
            if (m_verbose) std::cout << "SchmittTrigger: Stable low "  << std::endl;
        }
    }
    else
    {
        // check for transition - if valid over a timeframe, then switch
        if (m_rawValue <= m_lowValueThreshold)
        {
            if (m_timer > m_stableOFFTime)
            {
                // fall to low
                m_currentState = false;
                if (m_verbose) std::cout << "SchmittTrigger: Falling low "  << std::endl;
            }
            else
            {
                // wait for timer
                m_timer += (currentTime - m_previousTime);
                if (m_verbose) std::cout << "SchmittTrigger: I'm high and waiting "  << std::endl;
            }
        }
        else
        {
            // stable high - reset timer
            m_timer = 0;
            if (m_verbose) std::cout << "SchmittTrigger: Stable high "  << std::endl;
        }
    }
    m_previousTime = currentTime;
}

}

