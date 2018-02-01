/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
#ifndef _DIRL_TORQUE_REGRESSOR_
#define _DIRL_TORQUE_REGRESSOR_

#include "dynamicRegressorInterface.hpp"

namespace iDynTree{
    class SensorsList;
    class SensorsMeasurements;
}

namespace KDL {
namespace CoDyCo {

//class SensorsTree;

namespace Regressors {
/** \todo fix the case where the dynamic base has changed */
class torqueRegressor : public DynamicRegressorInterface
{
    const KDL::CoDyCo::UndirectedTree * p_undirected_tree;
    const iDynTree::SensorsList * p_sensors_tree;

    std::vector< int > subtree_leaf_links_indices; /** indices of the leafs (excluding the root) */

    std::vector<int> linkIndices2regrCols;

    std::string torque_dof;

    bool reverse_direction;

    std::vector<bool> activated_ft_sensors;

    bool consider_ft_offset;

    std::vector< int > subtree_links_indices; /** indices of the links belonging to the considered subtree */

    bool verbose;

    int torque_dof_index;

    int subtree_root_link_id;


    std::vector<int> relative_junction;

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
         *
         * @param _reverse_direction if true, reverse the direction of the regressor (root to joint instead of leaf to joint) default:false
         */
        torqueRegressor(const KDL::CoDyCo::UndirectedTree & _undirected_tree,
                        const iDynTree::SensorsList & _sensors_tree,
                        const std::vector<int> & _linkIndices2regrCols,
                        const std::string & dof_name,
                        const bool _reverse_direction = false,
                        const std::vector<bool> & _activated_ft_sensors=std::vector< bool>(0),
                        const bool _consider_ft_offset=false,
                        const bool _verbose=true
                        )
                        :   p_undirected_tree(&_undirected_tree),
                                            p_sensors_tree(&_sensors_tree),
                                            linkIndices2regrCols(_linkIndices2regrCols),
                                            torque_dof(dof_name),
                                            reverse_direction(_reverse_direction),
                                            activated_ft_sensors(_activated_ft_sensors),
                                            consider_ft_offset(_consider_ft_offset),
                                            subtree_links_indices(0),
                                            verbose(_verbose),
                                            NrOfRealLinks_subtree(0)

        {
            assert(linkIndices2regrCols.size() == p_undirected_tree->getNrOfLinks());
            NrOfRealLinks_subtree = 0;
            for(int ll=0; ll < (int)linkIndices2regrCols.size(); ll++ ) { if( linkIndices2regrCols[ll] != -1 ) { NrOfRealLinks_subtree++; } }
            assert(NrOfRealLinks_subtree >= 0);
            assert(NrOfRealLinks_subtree <= (int)linkIndices2regrCols.size());
        }

        virtual ~torqueRegressor() {};

        bool isActiveFTSensor(const int ft_sensor_id) const;

        int configure();

        int getNrOfOutputs();

        virtual std::vector<int> getRelativeJunctions();

        virtual iDynTree::Regressors::DynamicsRegressorParametersList getUsedParameters();

        virtual bool setGlobalParameters(const iDynTree::Regressors::DynamicsRegressorParametersList& globalParameters);


        virtual int computeRegressor(const KDL::JntArray &q,
                              const KDL::JntArray &q_dot,
                              const KDL::JntArray &q_dotdot,
                              const std::vector<KDL::Frame> & X_dynamic_base,
                              const std::vector<KDL::Twist> & v,
                              const std::vector<KDL::Twist> & a,
                              const iDynTree::SensorsMeasurements & measured_wrenches,
                              const KDL::JntArray & measured_torques,
                              Eigen::MatrixXd & regressor_matrix,
                              Eigen::VectorXd & known_terms);


};

}

}

}
#endif
