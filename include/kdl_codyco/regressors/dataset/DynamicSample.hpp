/*
 * Copyright (C) 2013 Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef __KDL_CODYCO_REGRESSOR_DYNAMIC_SAMPLE__
#define __KDL_CODYCO_REGRESSOR_DYNAMIC_SAMPLE__

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/kinfam.hpp>
#include <Eigen/Core>

#include <vector>

/*! \file DynamicSample.hpp define structures for state and sensor sample representation */

namespace KDL {
namespace CoDyCo {
namespace Regressors {

/**
 *
 * A generic interface to dynamic identification offline dataset
 */
class DynamicStateSample {
private:
    int n_dof;

public:
    double timestamp;
    KDL::Frame world_base_transform;
    KDL::Twist base_vel;
    KDL::Twist base_acc;
    KDL::JntArray q;
    KDL::JntArray dq;
    KDL::JntArray ddq;

    /**
     * DynamicStateSample constructor
     * @param[in] NrOfDOFs the number of degrees of freedom of the robot
     * @param[in] ts the timestamp of the sample (in seconds)
     */
    DynamicStateSample(const int NrOfDOFs=0, double ts = -1.0);

    DynamicStateSample(const DynamicStateSample & sample);

    ~DynamicStateSample();

    int getNrOfDOFs() const;

    bool setTimestamp(const double ts);

    double getTimestamp() const;

    /**
     * Get the world base transform \f[ {}^w H_b \f]
     */
    KDL::Frame getWorldBaseTransform() const;

    bool setWorldBaseTransform(const KDL::Frame & world_base_frame);

    /**
     * Get the base twist expressed in base reference frame \f[ {}^b v_b \f]
     */
    KDL::Twist getBaseVelocity() const;

    bool setBaseVelocity(const KDL::Twist & base_velocity);


    /**
     * Get the base proper (i.e. including gravitational acceleration) and spatial (i.e. not classical) acceleration \f[ {}^b a_b \f]
     */
    KDL::Twist getBaseSpatialAcceleration() const;

    /**
     * Get the base proper (i.e. including gravitational acceleration) and classical (i.e. not spatial) acceleration \f[ {}^b a_b \f]
     */
    KDL::Twist getBaseClassicalAcceleration() const;

    bool setBaseSpatialAcceleration(const KDL::Twist & spatial_base_acc);

    /**
     * Get the base proper (i.e. including gravitational acceleration) and classical (i.e. not spatial) acceleration \f[ {}^b a_b \f]
     * \note the classical -> spatial acceleration conversion is dependent on the base velocity, so it is important to set the base velocity before calling this method
     */
    bool setBaseClassicalAcceleration(const KDL::Twist & classical_base_acc);

    /**
     * Get the joint positions \f[ \mathbf{q} \f]
     */
    const KDL::JntArray & getJointPosition() const;

    bool setJointPosition(const KDL::JntArray & q);

    bool setJointPosition(double _q, int dof_id);

    /**
     * Get the joint velocities \f[ \dot{\mathbf{q}} \f]
     */
    const KDL::JntArray & getJointVelocity() const;

    bool setJointVelocity(const KDL::JntArray & dq);

    bool setJointVelocity(double _dq, int dof_id);

    /**
     * Get the joint accelerations \f[ \ddot{\mathbf{q}} \f]
     */
    const KDL::JntArray & getJointAcceleration() const;

    bool setJointAcceleration(const KDL::JntArray & ddq);

    bool setJointAcceleration(double _ddq, int dof_id);

};

/**
 *
 * A generic interface to dynamic identification offline dataset
 */

class DynamicSensorSample {
private:
    int nrOfTorqueSensors;
    int nrOfWrenchSensors;
    int nrOfThreeAxisFTSensors;

public:
    KDL::JntArray torque_sensors_measures;
    std::vector<KDL::Wrench> wrench_sensors_measures;
    std::vector<Eigen::Vector3d> three_axis_ft_sensors_measures;

    /**
     * DynamicStateSample constructor
     * @param[in] NrOfTorqueSensors the number of torque sensors in the sample
     * @param[in] NrOfWrenchSensors the number of wrench sensors in the sample
     * @param[in] NrOf3AxisFTSensors the number of 3 axis force/torque sensors (FSRs, load cells) in the sample
     */

    DynamicSensorSample(const int NrOfTorqueSensors = 0, const int NrOfWrenchSensors = 0, const int NrOf3AxisFTSensors = 0);

    DynamicSensorSample(const DynamicSensorSample & sample);

    ~DynamicSensorSample();

    int getNrOfWrenchSensors() const;

    int getNrOfTorqueSensors() const;

    int getNrOfThreeAxisForceTorqueSensors() const;

    int setNrOfWrenchSensors(int nr);

    int setNrOfTorqueSensors(int nr);

    int setNrOfThreeAxisForceTorqueSensors(int nr);

    bool setTorqueMeasure(const double measured_torque, const int torque_measured_id);

    double getTorqueMeasure(const int torque_measured_id) const;

    bool setWrenchMeasure(const KDL::Wrench measured_ft, const int wrench_measured_id);

    KDL::Wrench getWrenchMeasure(const int wrench_measured_id) const;

    bool setThreeAxisForceTorqueMeasure(const Eigen::Vector3d measured_three_axis_ft, const int three_axis_ft_measured_id);

    Eigen::Vector3d getThreeAxisForceTorqueMeasure(const int three_axis_ft_measured_id) const;

};



class DynamicSample: public DynamicStateSample, public DynamicSensorSample {
public:
    DynamicSample(const int NrOfDOFs=0,
                  const int NrOfTorqueSensors=0,
                  const int NrOfWrenchSensors=0,
                  const int NrOf3AxisFTSensors=0);

    DynamicSample(const DynamicSample & sample);

    ~DynamicSample();

};

}

}

}


#endif
