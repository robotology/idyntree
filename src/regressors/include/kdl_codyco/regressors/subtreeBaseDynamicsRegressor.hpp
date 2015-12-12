/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 */

#ifndef _DIRL_SUBTREE_ARTICULATED_DYNAMICS_REGRESSOR_
#define _DIRL_SUBTREE_ARTICULATED_DYNAMICS_REGRESSOR_

#include <kdl/jntarray.hpp>

#include <kdl_codyco/undirectedtree.hpp>
#include "dynamicRegressorInterface.hpp"

#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Core/Transform.h>

namespace KDL {
namespace CoDyCo {

class SensorsTree;

namespace Regressors {

class subtreeBaseDynamicsRegressor : public DynamicRegressorInterface
{
    const KDL::CoDyCo::UndirectedTree * p_undirected_tree;
    const iDynTree::SensorsList * p_sensors_tree;

    const std::vector<int> linkIndeces2regrCols;

    std::vector< std::string > subtree_leaf_links;

    bool consider_ft_offset;

    enum
    {
        USE_DYNAMIC_BASE_PLUCKER_FRAME,
        USE_FIRST_FT_SENSOR_PLUCKER_FRAME
    } regressor_plucker_frame;
    iDynTree::Transform sensor_H_parent_link;
    int first_ft_sensor_parent_link_id;

    std::vector< int > subtree_links_indices; /** indeces of the links belonging to the considered subtree */

    std::vector< int > subtree_leaf_links_indeces;

    bool verbose;

    std::vector< int > relative_junctions;

    int NrOfRealLinks_subtree;

    int isSubtreeLeaf(const int link_id) const;

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
         * Constructor for subtree articulated dynamics regressor
         *
         * @param _subtree_leaf_links the list of name of the leaf links of the considered subtree
         */
        subtreeBaseDynamicsRegressor(const KDL::CoDyCo::UndirectedTree & _undirected_tree,
                                     const iDynTree::SensorsList & _sensors_tree,
                                     const std::vector<int> & _linkIndeces2regrCols,
                                     std::vector< std::string> _subtree_leaf_links=std::vector< std::string>(0),
                                     const bool _consider_ft_offset=false,
                                     bool _verbose=true):
                                            p_undirected_tree(&_undirected_tree),
                                            p_sensors_tree(&_sensors_tree),
                                            linkIndeces2regrCols(_linkIndeces2regrCols),
                                            subtree_leaf_links(_subtree_leaf_links),
                                            consider_ft_offset(_consider_ft_offset),
                                            subtree_links_indices(0),
                                            verbose(_verbose),
                                            NrOfRealLinks_subtree(0)
        {
            assert(linkIndeces2regrCols.size() == p_undirected_tree->getNrOfLinks());
            NrOfRealLinks_subtree = 0;
            for(int ll=0; ll < (int)linkIndeces2regrCols.size(); ll++ ) { if( linkIndeces2regrCols[ll] != -1 ) { NrOfRealLinks_subtree++; } }
            assert(NrOfRealLinks_subtree >= 0);
            assert(NrOfRealLinks_subtree <= (int)linkIndeces2regrCols.size());
        }

        virtual ~subtreeBaseDynamicsRegressor() {};

        virtual int getNrOfOutputs();

        virtual std::vector<int> getRelativeJunctions();

        virtual iDynTree::Regressors::DynamicsRegressorParametersList getUsedParameters();

        virtual bool setGlobalParameters(const iDynTree::Regressors::DynamicsRegressorParametersList & globalParameters);

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
