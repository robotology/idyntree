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

#include <iDynTree/Sensors/Sensors.hpp>
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

    const std::vector<int> linkIndeces2regrCols;

    bool verbose;

    int NrOfRealLinks_subtree;




    public:
        /**
         * Constructor for base dynamics regressor
         *
         */
        baseDynamicsRegressor(const KDL::CoDyCo::UndirectedTree & _undirected_tree,
                              const std::vector<int> & _linkIndeces2regrCols,
                                     bool _verbose=true):
                                            p_undirected_tree(&_undirected_tree),
                                            linkIndeces2regrCols(_linkIndeces2regrCols),
                                            verbose(_verbose)
        {
            assert(linkIndeces2regrCols.size() == p_undirected_tree->getNrOfLinks());
            NrOfRealLinks_subtree = 0;
            for(int ll=0; ll < (int)linkIndeces2regrCols.size(); ll++ ) { if( linkIndeces2regrCols[ll] != -1 ) { NrOfRealLinks_subtree++; } }
            assert(NrOfRealLinks_subtree >= 0);
            assert(NrOfRealLinks_subtree <= (int)linkIndeces2regrCols.size());
        }

        virtual ~baseDynamicsRegressor() {};

        int getNrOfOutputs();

        std::vector<int> getRelativeJunctions();

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
