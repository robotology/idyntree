/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
#ifndef _DIRL_DYNAMIC_REGRESSOR_INTERFACE_
#define _DIRL_DYNAMIC_REGRESSOR_INTERFACE_

#include <kdl_codyco/undirectedtree.hpp>

#include <iDynTree/Regressors/DynamicsRegressorParameters.h>
#include <kdl/jntarray.hpp>
#include <iDynTree/Sensors/Sensors.h>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace iDynTree {
 class SensorsMeasurements;
}


namespace KDL {
namespace CoDyCo {


namespace Regressors {

class DynamicRegressorInterface {

public:
    virtual ~DynamicRegressorInterface(){}

    virtual int getNrOfOutputs() = 0;

    virtual std::vector<int> getRelativeJunctions() = 0;

    virtual iDynTree::Regressors::DynamicsRegressorParametersList getUsedParameters() = 0;

    virtual bool setGlobalParameters(const iDynTree::Regressors::DynamicsRegressorParametersList & globalParameters) = 0;

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
                                 const iDynTree::SensorsMeasurements & sensors_measurements,  // measurments
                                 const KDL::JntArray & measured_torques,
                                 Eigen::MatrixXd & regressor_matrix,                    //output
                                 Eigen::VectorXd & known_terms) = 0;

};

}

}

}
#endif
