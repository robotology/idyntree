/*
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "iDynTree/Estimation/robotStatus.h"

#include <iCub/iDynTree/yarp_kdl.h>

namespace iDynTree
{

    RobotJointStatus::RobotJointStatus(int nrOfDOFs)
    {
        setNrOfDOFs(nrOfDOFs);

        this->zero();
    }

    bool RobotJointStatus::setNrOfDOFs(int nrOfDOFs)
    {

        qj.resize(nrOfDOFs);
        dqj.resize(nrOfDOFs);
        ddqj.resize(nrOfDOFs);
        torquesj.resize(nrOfDOFs);

        qj_kdl.resize(nrOfDOFs);
        dqj_kdl.resize(nrOfDOFs);
        ddqj_kdl.resize(nrOfDOFs);
        torquesj_kdl.resize(nrOfDOFs);

        return zero();
    }

    bool RobotJointStatus::zero()
    {
        qj.zero();
        dqj.zero();
        ddqj.zero();
        torquesj.zero();

        SetToZero(qj_kdl);
        SetToZero(dqj_kdl);
        SetToZero(ddqj_kdl);
        SetToZero(torquesj_kdl);

        return true;
    }

    bool RobotJointStatus::setJointPosYARP(const yarp::sig::Vector& _qj)
    {
        qj = _qj;
        return YarptoKDL(qj,qj_kdl);
    }

    bool RobotJointStatus::setJointVelYARP(const yarp::sig::Vector& _dqj)
    {
        dqj = _dqj;
        return YarptoKDL(dqj,dqj_kdl);
    }

    bool RobotJointStatus::setJointAccYARP(const yarp::sig::Vector& _ddqj)
    {
        ddqj = _ddqj;
        return YarptoKDL(ddqj,ddqj_kdl);
    }

    bool RobotJointStatus::setJointTorquesYARP(const yarp::sig::Vector& _torquesj)
    {
        torquesj = _torquesj;
        return YarptoKDL(torquesj,torquesj_kdl);
    }


    bool RobotJointStatus::setJointPosKDL(const KDL::JntArray& _qj)
    {
        qj_kdl = _qj;
        return KDLtoYarp(qj_kdl,qj);
    }

    bool RobotJointStatus::setJointVelKDL(const KDL::JntArray& _dqj)
    {
        dqj_kdl = _dqj;
        return KDLtoYarp(dqj_kdl,dqj);
    }

    bool RobotJointStatus::setJointAccKDL(const KDL::JntArray& _ddqj)
    {
        ddqj_kdl = _ddqj;
        return KDLtoYarp(ddqj_kdl,ddqj);
    }

    bool RobotJointStatus::setJointTorquesKDL(const KDL::JntArray& _torquesj)
    {
        torquesj_kdl = _torquesj;
        return KDLtoYarp(torquesj_kdl,torquesj);
    }

    KDL::JntArray& RobotJointStatus::getJointPosKDL()
    {
        return qj_kdl;
    }

    KDL::JntArray& RobotJointStatus::getJointVelKDL()
    {
        return dqj_kdl;
    }

    KDL::JntArray& RobotJointStatus::getJointAccKDL()
    {
        return ddqj_kdl;
    }

    KDL::JntArray& RobotJointStatus::getJointTorquesKDL()
    {
        return torquesj_kdl;
    }

    yarp::sig::Vector& RobotJointStatus::getJointPosYARP()
    {
        return qj;
    }

    yarp::sig::Vector& RobotJointStatus::getJointVelYARP()
    {
        return dqj;
    }

    yarp::sig::Vector& RobotJointStatus::getJointAccYARP()
    {
        return ddqj;
    }

    yarp::sig::Vector& RobotJointStatus::getJointTorquesYARP()
    {
        return torquesj;
    }

    bool RobotJointStatus::updateYarpBuffers()
    {
        bool ok = true;
        ok = ok && KDLtoYarp(qj_kdl,qj);
        ok = ok && KDLtoYarp(dqj_kdl,dqj);
        ok = ok && KDLtoYarp(ddqj_kdl,ddqj);
        ok = ok && KDLtoYarp(torquesj_kdl,torquesj);
        return ok;
    }

    bool RobotJointStatus::updateKDLBuffers()
    {
        bool ok = true;
        ok = ok && YarptoKDL(qj,qj_kdl);
        ok = ok && YarptoKDL(dqj,dqj_kdl);
        ok = ok && YarptoKDL(ddqj,ddqj_kdl);
        ok = ok && YarptoKDL(torquesj,torquesj_kdl);
        return ok;
    }
    
}

//RobotSensorStatus::RobotSensorStatus(int nrOfFTSensors)
//{
//    setNrOfFTSensors(nrOfFTSensors);
//
//    this->zero();
//}
//
//
//bool RobotSensorStatus::zero()
//{
//    domega_imu.zero();
//    omega_imu.zero();
//    proper_ddp_imu.zero();
//    wbi_imu.zero();
//    for(unsigned int i=0; i < estimated_ft_sensors.size(); i++ ) {
//        estimated_ft_sensors[i].zero();
//        measured_ft_sensors[i].zero();
//        ft_sensors_offset[i].zero();
//        model_ft_sensors[i].zero();
//    }
//    return true;
//}
//
//bool RobotSensorStatus::setNrOfFTSensors(int nrOfFTsensors)
//{
//    domega_imu.resize(3);
//    omega_imu.resize(3);
//    proper_ddp_imu.resize(3);
//    wbi_imu.resize(wbi::sensorTypeDescriptions[wbi::SENSOR_IMU].dataSize);
//
//    estimated_ft_sensors.resize(nrOfFTsensors,yarp::sig::Vector(6,0.0));
//    measured_ft_sensors.resize(nrOfFTsensors,yarp::sig::Vector(6,0.0));
//    ft_sensors_offset.resize(nrOfFTsensors,yarp::sig::Vector(6,0.0));
//    model_ft_sensors.resize(nrOfFTsensors,yarp::sig::Vector(6,0.0));
//    zero();
//    return true;
//}
