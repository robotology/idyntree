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

#ifndef IDYNTREE_SCHMITT_TRIGGER_H
#define IDYNTREE_SCHMITT_TRIGGER_H

#include <iostream>
namespace iDynTree
{
   /**
    * Schmitt Trigger class for binary state detection 
    * This device is used to obtain a binary state (ON/OFF) at 
    * some time instant depending on a value inputted to the device.
    * It is initialized with 4 parameters,
    *
    * |     Parameter    | Type |                              Description                             |
    * |:----------------:|:----:|:--------------------------------------------------------------------:|
    * |stableOFFTime     |double|Time to elapse to switch to OFF state, once low threshold is triggered|
    * |stableONTime      |double|Time to elapse to switch to ON state, once high threshold is triggered|
    * |lowValueThreshold |double|                 Threshold Value to turn state OFF                    |
    * |highValueThreshold|double|                 Threshold Value to turn state ON                     |
    * 
    * The device is first configured with the user-defined parameter settings, but is not activated until 
    * the first call of updateDevice method. The updateDevice method must be called at every iteration with 
    * the current time and the value to be compared, for continuously updating the binary state of the device.
    * 
    * For instance, if the current state is OFF and there is a call to updateDevice with current time and an input 
    * value greater than the highValueThreshold, then a timer is activated and is updated at every call to updateMethod.
    * If there are consecutive calls to the updateMethod with input values greater than the highValueThreshold,
    * until the timer lapses stableONTime, then the state is switched to ON, otherwise it is remains OFF.
    * 
    * NOTE: Time is specified relative to the process that is instantiating this device.
    * Default state is set to ON during instantiation. This can be changed accordingly using the setInitialState method.
    *
    * NOTE: There are no default parameters to the Schmitt Trigger. These parameters are set through 
    * the constructor during instantiation.
    */
    class SchmittTrigger
    {
    public:
        /**
         * Constructor
         * @param stableOFFTime time to elapse to switch to OFF state
         * @param stableONTime time to elapse to switch to ON state
         * @param lowValueThreshold threshold value to turn state OFF
         * @param highValueThreshold threshold value to turn state ON
         */
        SchmittTrigger(double stableOFFTime, 
                       double stableONTime, 
                       double lowValueThreshold, 
                       double highValueThreshold);
               
        /**
         * Sets the schmitt trigger settings to default values
         */
        void resetDevice();
        
        /**
         * Update the device state with the latest time and input measurement
         * @param currentTime current time
         * @param rawValue input measurement
         */
        void updateDevice(double currentTime, double rawValue);
        
        /**
         * @name Setters
         */
        //@{
        
        /**
         * Configures the Schmitt Trigger parameters 
         * (can be called after instantiation, to change the parameters)
         * @param stableOFFTime time to elapse to switch to OFF state
         * @param stableONTime time to elapse to switch to ON state
         * @param lowValueThreshold threshold value to turn state OFF
         * @param highValueThreshold threshold value to turn state ON
         */
        void configure(double stableOFFTime, 
                       double stableONTime, 
                       double lowValueThreshold, 
                       double highValueThreshold);
        
        /**
         * set required time to elapse to switch to OFF state
         * @param stableOFFTime 
         */
        void setStableOFFTime(double stableOFFTime) { m_stableOFFTime = stableOFFTime; }
        
        /**
         * set required time to elapse to switch to ON state
         * @param stableONTime 
         */
        void setStableONTime(double stableONTime) { m_stableONTime = stableONTime; }

        /**
         * set low threshold value to trigger OFF state detection
         * @param lowValueThreshold 
         */
        void setLowValueThreshold(double lowValueThreshold) { m_lowValueThreshold = lowValueThreshold; }
        
        /**
         * set high threshold value to trigger ON state detection
         * @param highValueThreshold 
         */
        void setHighValueThreshold(double highValueThreshold) { m_highValueThreshold = highValueThreshold; }

        /**
         * set initial state
         * @param state binary state true/false
         */
        void setInitialState(bool state) { m_currentState = state; }
        //@}
    
        /**
         * @name Getters
         */
        //@{
        
        /**
         * get current state
         * @return binary state true/false
         */
        bool getState() { return m_currentState; }
        
        /**
         * get time elapsed since the first update of the device
         * @return time
         */
        double getElapsedTime() { return m_previousTime; }
        //@}
        
        
        /**
         * @name Verbose
         */
        //@{
        void setVerbose() {m_verbose = 1;}
        void unsetVerbose() {m_verbose = 0;}
        //@}
    private:
   
        /**
         * @name State 
         */
        //@{
        bool m_currentState;
        double m_previousTime;
        double m_timer;
        //@}
        
         /**
         * @name Device Parameters
         */
        //@{
        double m_stableOFFTime;
        double m_stableONTime;
        double m_highValueThreshold;
        double m_lowValueThreshold; 
        //@}
        
        /**
         * @name Input
         */
        //@{   
        double m_rawValue;
        //@}
        
        
        // Verbose flag
        int m_verbose;

    };
}

#endif