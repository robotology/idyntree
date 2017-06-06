/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 */

#ifndef _DIRL_BASE_DYNAMICS_REGRESSOR_
#define _DIRL_BASE_DYNAMICS_REGRESSOR_

#include <kdl/jntarray.hpp>

#include "kdl_codyco/undirectedtree.hpp"
#include "dynamicRegressorInterface.hpp"

#include <iDynTree/Sensors/Sensors.h>
namespace iDynTree{
    class SensorsList;
}

namespace KDL {
namespace CoDyCo {



namespace Regressors {

class baseDynamicsRegressor : public DynamicRegressorInterface
{
    const KDL::CoDyCo::UndirectedTree * p_undirected_tree;
    const iDynTree::SensorsList * p_sensors_tree;

    const std::vector<int> linkIndices2regrCols;

    bool verbose;

    int NrOfRealLinks_subtree;

    Eigen::MatrixXd regressor_local_parametrization;

    // This regressor computes the regressor assuming that the
    // a full 10*N_Links + (if ft offset is enabled) 6*N_ft
    // parameters, in the order specified by the link and ft
    // sensors serialization. The output regressor can have
    // an arbitrary serialization, and so we keep in this
    // vector the mapping between localParametersIndices
    // (from [0 to 10*N_Links + (6*N_FT) - 1)] to [0 to getNrOfParameters]
    std::vector<int> localParametersIndexToOutputParametersIndex;

    public:
        /**
         * Constructor for base dynamics regressor
         *
         */
        baseDynamicsRegressor(const KDL::CoDyCo::UndirectedTree & _undirected_tree,
                              const std::vector<int> & _linkIndices2regrCols,
                                     bool _verbose=true):
                                            p_undirected_tree(&_undirected_tree),
                                            linkIndices2regrCols(_linkIndices2regrCols),
                                            verbose(_verbose)
        {
            assert(linkIndices2regrCols.size() == p_undirected_tree->getNrOfLinks());
            NrOfRealLinks_subtree = 0;
            for(int ll=0; ll < (int)linkIndices2regrCols.size(); ll++ ) { if( linkIndices2regrCols[ll] != -1 ) { NrOfRealLinks_subtree++; } }
            assert(NrOfRealLinks_subtree >= 0);
            assert(NrOfRealLinks_subtree <= (int)linkIndices2regrCols.size());
        }

        virtual ~baseDynamicsRegressor() {};

        virtual int getNrOfOutputs();

        virtual std::vector<int> getRelativeJunctions();

        virtual iDynTree::Regressors::DynamicsRegressorParametersList getUsedParameters();
        virtual bool setGlobalParameters(const iDynTree::Regressors::DynamicsRegressorParametersList& globalParameters);

        int computeRegressor(const KDL::JntArray &q,
                              const KDL::JntArray &q_dot,
                              const KDL::JntArray &q_dotdot,
                              const std::vector<KDL::Frame> & X_dynamic_base,
                              const std::vector<KDL::Twist> &v,
                              const std::vector<KDL::Twist> & a,
                              const iDynTree::SensorsMeasurements & measured_wrenches,
                              const KDL::JntArray & measured_torques,
                              Eigen::MatrixXd & regressor_matrix,
                              Eigen::VectorXd & known_terms);

        int configure();


};

}

}

}
#endif
