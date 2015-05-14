/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 */

#ifndef _DIRL_DYNAMIC_REGRESSOR_INTERFACE_
#define _DIRL_DYNAMIC_REGRESSOR_INTERFACE_

#include <kdl_codyco/undirectedtree.hpp>
#include "../../iDynTree/Regressors/DynamicsRegressorParameters.h"
#include <kdl/jntarray.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace KDL {
namespace CoDyCo {

class SensorsMeasurements;

namespace Regressors {

class DynamicRegressorInterface {

public:
    virtual ~DynamicRegressorInterface(){}

    virtual int getNrOfOutputs() = 0;

    virtual std::vector<int> getRelativeJunctions() = 0;

    virtual std::vector<iDynTree::Regressors::DynamicsRegressorParameterType> getUsedParameters();

    /**
     * Configure the regressor given a UndirectedTree (for example allocating
     * the necessary datastructures)
     *
     * Return 0 if all went well, a negative number otherwise
     *
     */
    virtual int configure() = 0;

    /**
     * Given a robot state, compute the regressor
     *
     */
    virtual int computeRegressor(const KDL::JntArray &q,                                //state
                                 const KDL::JntArray &q_dot,
                                 const KDL::JntArray &q_dotdot,
                                 const std::vector<KDL::Frame> & X_dynamic_base,        //result of forward kinematic
                                 const std::vector<KDL::Twist> &v,
                                 const std::vector<KDL::Twist> &a,
                                 const KDL::CoDyCo::SensorsMeasurements & sensors_measurements,  // measurments
                                 const KDL::JntArray & measured_torques,
                                 Eigen::MatrixXd & regressor_matrix,                    //output
                                 Eigen::VectorXd & known_terms) = 0;

};

}

}

}
#endif
