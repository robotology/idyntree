/*
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
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

#ifndef ESTIMATION_ROBOT_STATUS_H_
#define ESTIMATION_ROBOT_STATUS_H_

#include <yarp/sig/Vector.h>

#include <kdl/jntarray.hpp>

#include <vector>

namespace iDynTree {
    
    class RobotJointStatus
    {
        yarp::sig::Vector qj;
        yarp::sig::Vector dqj;
        yarp::sig::Vector ddqj;
        yarp::sig::Vector torquesj;

        KDL::JntArray qj_kdl;
        KDL::JntArray dqj_kdl;
        KDL::JntArray ddqj_kdl;
        KDL::JntArray torquesj_kdl;

    public:
        bool zero();
        RobotJointStatus(int nrOfDOFs=0);
        bool setNrOfDOFs(int nrOfDOFs);

        bool setJointPosYARP(const yarp::sig::Vector & qj);
        bool setJointVelYARP(const yarp::sig::Vector & dqj);
        bool setJointAccYARP(const yarp::sig::Vector & ddqj);
        bool setJointTorquesYARP(const yarp::sig::Vector & torquesj);

        bool setJointPosKDL(const KDL::JntArray & qj);
        bool setJointVelKDL(const KDL::JntArray & dqj);
        bool setJointAccKDL(const KDL::JntArray & ddqj);
        bool setJointTorquesKDL(const KDL::JntArray & torquesj);

        yarp::sig::Vector & getJointPosYARP();
        yarp::sig::Vector & getJointVelYARP();
        yarp::sig::Vector & getJointAccYARP();
        yarp::sig::Vector & getJointTorquesYARP();

        KDL::JntArray & getJointPosKDL();
        KDL::JntArray & getJointVelKDL();
        KDL::JntArray & getJointAccKDL();
        KDL::JntArray & getJointTorquesKDL();

        /**
         * Copy in the yarp buffers the content of the KDL buffers.
         */
        bool updateYarpBuffers();

        /**
         * Copy in the KDL buffers the content of the YARP buffers.
         */
        bool updateKDLBuffers();
    };
    
}
//class RobotSensorStatus
//{
//public:
//    yarp::sig::Vector omega_imu;
//    yarp::sig::Vector domega_imu;
//    yarp::sig::Vector proper_ddp_imu;
//    yarp::sig::Vector wbi_imu;
//
//    std::vector<yarp::sig::Vector> measured_ft_sensors;
//    std::vector<yarp::sig::Vector> estimated_ft_sensors;
//    std::vector<yarp::sig::Vector> ft_sensors_offset;
//    std::vector<yarp::sig::Vector> model_ft_sensors;
//
//    RobotSensorStatus(int nrOfFTSensors=0);
//    bool setNrOfFTSensors(int nrOfFTSensors);
//    bool zero();
//};

#endif