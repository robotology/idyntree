/**
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 */

#include "kdl_codyco/regressors/dirl_utils.hpp"
#include "baseDynamicsRegressor.hpp"

#include <kdl_codyco/regressor_utils.hpp>

#include <iostream>

using namespace KDL::CoDyCo;

namespace KDL {
namespace CoDyCo {
namespace Regressors {

int baseDynamicsRegressor::configure()
{
    iDynTree::Regressors::DynamicsRegressorParametersList localSerialization = getLegacyUsedParameters(linkIndices2regrCols);

    regressor_local_parametrization.resize(6,localSerialization.parameters.size());

    return 0;
}

int baseDynamicsRegressor::getNrOfOutputs()
{
    return 6;
}

std::vector<int> baseDynamicsRegressor::getRelativeJunctions()
{
    return std::vector<int>(0);
}


iDynTree::Regressors::DynamicsRegressorParametersList baseDynamicsRegressor::getUsedParameters()
{
    assert(p_undirected_tree->getNrOfLinks() == linkIndices2regrCols.size());

    return getLegacyUsedParameters(linkIndices2regrCols);
}


int baseDynamicsRegressor::computeRegressor(const KDL::JntArray &q,
                                      const KDL::JntArray &q_dot,
                                      const KDL::JntArray &q_dotdot,
                                      const std::vector<KDL::Frame> & X_dynamic_base,
                                      const std::vector<KDL::Twist> & v,
                                      const std::vector<KDL::Twist> & a,
                                      const iDynTree::SensorsMeasurements & measured_wrenches,
                                      const KDL::JntArray & measured_torques,
                                      Eigen::MatrixXd & regressor_matrix_global_column_serialization,
                                      Eigen::VectorXd & known_terms)
{
#ifndef NDEBUG
    //std::cerr << "Called computeRegressor " << std::endl;
    //std::cerr << (*p_ft_list).toString();
#endif
    const KDL::CoDyCo::UndirectedTree & undirected_tree = *p_undirected_tree;


    if( regressor_local_parametrization.rows() != getNrOfOutputs() ) {
        return -1;
    }

    //all other columns, beside the one relative to the inertial parameters of the links of the subtree, are zero
    regressor_local_parametrization.setZero();

    for(int link_id =0; link_id < (int)undirected_tree.getNrOfLinks(); link_id++ ) {

        if( linkIndices2regrCols[link_id] != -1 ) {
            Eigen::Matrix<double,6,10> netWrenchRegressor_i = netWrenchRegressor(v[link_id],a[link_id]);
            regressor_local_parametrization.block(0,(int)(10*linkIndices2regrCols[link_id]),getNrOfOutputs(),10) = WrenchTransformationMatrix(X_dynamic_base[link_id])*netWrenchRegressor_i;
        }
    }

    convertLocalRegressorToGlobalRegressor(regressor_local_parametrization,regressor_matrix_global_column_serialization,this->localParametersIndexToOutputParametersIndex);

    return 0;
}

bool baseDynamicsRegressor::setGlobalParameters(const iDynTree::Regressors::DynamicsRegressorParametersList& globalParameters)
{
    iDynTree::Regressors::DynamicsRegressorParametersList localSerialiaziation =
        getLegacyUsedParameters(linkIndices2regrCols);

    buildParametersMapping(localSerialiaziation,globalParameters,this->localParametersIndexToOutputParametersIndex);

    return true;
}

}

}

}
