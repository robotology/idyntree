/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/kinfam.hpp>

#include "DynamicSample.hpp"

namespace KDL {
namespace CoDyCo {
namespace Regressors {

DynamicStateSample::DynamicStateSample(const int NrOfDOFs, const double ts): n_dof(NrOfDOFs),
                                                                             timestamp(ts),
                                                                             q(NrOfDOFs),
                                                                             dq(NrOfDOFs),
                                                                             ddq(NrOfDOFs)
{

}

DynamicStateSample::DynamicStateSample(const DynamicStateSample & sample):  n_dof(sample.getNrOfDOFs()),
                                                                            timestamp(sample.getTimestamp()),
                                                                            q(sample.getNrOfDOFs()),
                                                                            dq(sample.getNrOfDOFs()),
                                                                            ddq(sample.getNrOfDOFs())
{
    setWorldBaseTransform(sample.getWorldBaseTransform());
    setBaseVelocity(sample.getBaseVelocity());
    setBaseSpatialAcceleration(sample.getBaseSpatialAcceleration());
    setJointPosition(sample.getJointPosition());
    setJointVelocity(sample.getJointVelocity());
    setJointAcceleration(sample.getJointAcceleration());
}

DynamicStateSample::~DynamicStateSample()
{
}

bool DynamicStateSample::setTimestamp(const double ts)
{
    timestamp = ts;
    return true;
}

double DynamicStateSample::getTimestamp() const
{
    return timestamp;
}

int DynamicStateSample::getNrOfDOFs() const
{
    return n_dof;
}

KDL::Frame DynamicStateSample::getWorldBaseTransform() const
{
    return world_base_transform;
}

bool DynamicStateSample::setWorldBaseTransform(const KDL::Frame & _world_base_frame)
{
    world_base_transform = _world_base_frame;
    return true;
}


KDL::Twist DynamicStateSample::getBaseVelocity() const
{
    return base_vel;
}

bool DynamicStateSample::setBaseVelocity(const KDL::Twist & _base_vel)
{
    base_vel = _base_vel;
    return true;
}

KDL::Twist DynamicStateSample::getBaseSpatialAcceleration() const
{
    return base_acc;
}

bool DynamicStateSample::setBaseSpatialAcceleration(const KDL::Twist & _base_acc)
{
    base_acc = _base_acc;
    return true;
}


KDL::Twist DynamicStateSample::getBaseClassicalAcceleration() const
{
    /**
     * If you have doubt of this formula, check
     * Featherstone - Rigid Body Dynamics Algorithms - 2008 - Section 2.11
     *
     */
    KDL::Twist classical_base_acc;
    classical_base_acc.rot = base_acc.rot;
    classical_base_acc.vel = base_acc.vel-base_vel.vel*base_vel.rot;
    return classical_base_acc;
}

bool DynamicStateSample::setBaseClassicalAcceleration(const KDL::Twist & classical_base_acc)
{
    /**
     * If you have doubt of this formula, check
     * Featherstone - Rigid Body Dynamics Algorithms - 2008 - Section 2.11
     */
    base_acc.rot = classical_base_acc.rot;
    base_acc.vel = classical_base_acc.vel + base_vel.vel*base_vel.rot;
    return true;
}

const KDL::JntArray & DynamicStateSample::getJointPosition() const
{
    return q;
}

const KDL::JntArray & DynamicStateSample::getJointVelocity() const
{
    return dq;
}

const KDL::JntArray & DynamicStateSample::getJointAcceleration() const
{
    return ddq;
}

bool DynamicStateSample::setJointPosition(const KDL::JntArray & _q)
{
    if( _q.rows() != (int)getNrOfDOFs() ) { return false; };
    q = _q;
    return true;
}

bool DynamicStateSample::setJointPosition(double _q, int dof_id)
{
    if( dof_id < 0 || dof_id >= getNrOfDOFs() ) { return false; }
    q(dof_id) = _q;
    return true;
}


bool DynamicStateSample::setJointVelocity(const KDL::JntArray & _dq)
{
    if( _dq.rows() != (int)getNrOfDOFs() ) { return false; };
    dq = _dq;
    return true;
}

bool DynamicStateSample::setJointVelocity(double _dq, int dof_id)
{
    if( dof_id < 0 || dof_id >= (int)getNrOfDOFs() ) { return false; }
    dq(dof_id) = _dq;
    return true;
}


bool DynamicStateSample::setJointAcceleration(const KDL::JntArray & _ddq)
{
    if( _ddq.rows() != (int)getNrOfDOFs() ) { return false; };
    ddq = _ddq;
    return true;
}

bool DynamicStateSample::setJointAcceleration(double _ddq, int dof_id)
{
    if( dof_id < 0 || dof_id >= getNrOfDOFs() ) { return false; }
    ddq(dof_id) = _ddq;
    return true;
}



DynamicSensorSample::DynamicSensorSample(const int NrOfTorqueSensors,
                                         const int NrOfWrenchSensors,
                                         const int NrOfThreeAxisSensors
                                        ):
                                         nrOfTorqueSensors(NrOfTorqueSensors),
                                         nrOfWrenchSensors(NrOfWrenchSensors),
                                         nrOfThreeAxisFTSensors(NrOfThreeAxisSensors),
                                         torque_sensors_measures(NrOfTorqueSensors)
{
    wrench_sensors_measures.resize(NrOfWrenchSensors);
    three_axis_ft_sensors_measures.resize(NrOfThreeAxisSensors);
}

DynamicSensorSample::DynamicSensorSample(const DynamicSensorSample & sample):
                                         nrOfTorqueSensors(sample.getNrOfTorqueSensors()),
                                         nrOfWrenchSensors(sample.getNrOfWrenchSensors()),
                                         nrOfThreeAxisFTSensors(sample.getNrOfThreeAxisForceTorqueSensors()),
                                         torque_sensors_measures(sample.getNrOfTorqueSensors())
{
    wrench_sensors_measures.resize(sample.getNrOfWrenchSensors());
    three_axis_ft_sensors_measures.resize(sample.getNrOfThreeAxisForceTorqueSensors());

    for(int i=0; i < getNrOfTorqueSensors(); i++ ) {
        setTorqueMeasure(sample.getTorqueMeasure(i),i);
    }

    for(int i=0; i < getNrOfWrenchSensors(); i++ ) {
        setWrenchMeasure(sample.getWrenchMeasure(i),i);
    }

    for(int i=0; i < getNrOfThreeAxisForceTorqueSensors(); i++ ) {
        setThreeAxisForceTorqueMeasure(sample.getThreeAxisForceTorqueMeasure(i),i);
    }

}



DynamicSensorSample::~DynamicSensorSample()
{
}

int DynamicSensorSample::getNrOfWrenchSensors() const
{
    return nrOfWrenchSensors;
}

int DynamicSensorSample::getNrOfTorqueSensors() const
{
    return nrOfTorqueSensors;
}

int DynamicSensorSample::getNrOfThreeAxisForceTorqueSensors() const
{
    return nrOfThreeAxisFTSensors;
}

bool DynamicSensorSample::setTorqueMeasure(const double measured_torque, const int torque_measured_id)
{
    if( torque_measured_id < 0 || torque_measured_id >= nrOfTorqueSensors ) { return false; }
    torque_sensors_measures(torque_measured_id) = measured_torque;
    return true;
}

double DynamicSensorSample::getTorqueMeasure(const int torque_measured_id) const
{
    if( torque_measured_id < 0 || torque_measured_id >= nrOfTorqueSensors ) { return 0; }
    return torque_sensors_measures(torque_measured_id);
}

bool DynamicSensorSample::setWrenchMeasure(const KDL::Wrench measured_wrench, const int wrench_measured_id)
{
    if( wrench_measured_id < 0 || wrench_measured_id >= nrOfWrenchSensors ) { return false; }
    wrench_sensors_measures[wrench_measured_id] = measured_wrench;
    return true;
}

KDL::Wrench DynamicSensorSample::getWrenchMeasure(const int wrench_measured_id) const
{
    if( wrench_measured_id < 0 || wrench_measured_id >= nrOfWrenchSensors ) { return KDL::Wrench::Zero(); }
    return wrench_sensors_measures[wrench_measured_id];
}

bool DynamicSensorSample::setThreeAxisForceTorqueMeasure(const KDL::Vector measured_three_axis_ft, const int three_axis_ft_measured_id)
{
    if( three_axis_ft_measured_id < 0 || three_axis_ft_measured_id >= nrOfThreeAxisFTSensors ) { return false; }
    three_axis_ft_sensors_measures[three_axis_ft_measured_id] = measured_three_axis_ft;
    return true;
}

KDL::Vector DynamicSensorSample::getThreeAxisForceTorqueMeasure(const int three_axis_ft_measured_id) const
{
    if( three_axis_ft_measured_id < 0 || three_axis_ft_measured_id >= nrOfThreeAxisFTSensors ) { return KDL::Vector(); }
    return three_axis_ft_sensors_measures[three_axis_ft_measured_id];
}

DynamicSample::DynamicSample(const int NrOfDOFs,
                             const int NrOfTorqueSensors,
                             const int NrOfWrenchSensors,
                             const int NrOfThreeAxisSensors):
                             DynamicStateSample(NrOfDOFs),
                             DynamicSensorSample(NrOfTorqueSensors,NrOfWrenchSensors,NrOfThreeAxisSensors)
{
}

DynamicSample::DynamicSample(const DynamicSample & sample):
                             DynamicStateSample(sample),
                             DynamicSensorSample(sample)
{
}

DynamicSample::~DynamicSample()
{
}

}

}

}
