/**
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 */

#include "subtreeBaseDynamicsRegressor.hpp"

#include "kdl_codyco/regressor_utils.hpp"

#include "iDynTree/Sensors/SixAxisFTSensor.h"
#include "iDynTree/Core/Wrench.h"
#include "kdl_codyco/KDLConversions.h"
#include "dirl_utils.hpp"

#include <iostream>

using namespace KDL::CoDyCo;

namespace KDL {
namespace CoDyCo {
namespace Regressors {

int subtreeBaseDynamicsRegressor::isSubtreeLeaf(const int link_id) const
{
    for( int i=0; i < (int)subtree_leaf_links_indeces.size(); i++ ) {
        if( link_id == subtree_leaf_links_indeces[i] ) return true;
    }
    return false;
}

int subtreeBaseDynamicsRegressor::configure()
{
    const KDL::CoDyCo::UndirectedTree & undirected_tree = *p_undirected_tree;
    const iDynTree::SensorsList & sensors_tree = *p_sensors_tree;

    //Checking if the provided subtree leafs define a proper subtree
    subtree_leaf_links_indeces.resize(subtree_leaf_links.size());

    if( subtree_leaf_links.size() == 0 )
    {
        if( verbose )
        {
            std::cerr << "subtreeBaseDynamicsRegressor::configure error: subtree with no leaf not implemented"
                      << std::endl;
        }
        return -3;
    }

    for(int i=0; i < (int)subtree_leaf_links.size(); i++ )
    {
        LinkMap::const_iterator link_it = undirected_tree.getLink(subtree_leaf_links[i]);

        if( link_it == undirected_tree.getInvalidLinkIterator() )
        {
            if( verbose )
            {
                std::cerr << "subtreeBaseDynamicsRegressor::configure error: link "
                          << subtree_leaf_links[i] << " not found " << std::endl;
                std::cerr << "Link available in the model : " << std::endl;
                for(unsigned int i=0; i < undirected_tree.getNrOfLinks(); i++ )
                {
                    std::cerr << undirected_tree.getLink(i)->getName() << std::endl;
                }
            }
            return -1;
        }

        int nrOfFTSensorsAttachedToLink = getNrOfFTSensorsOnLink(sensors_tree,link_it->getLinkIndex());

        if( nrOfFTSensorsAttachedToLink < 0 )
        {
            if( verbose ) { std::cerr << "subtreeBaseDynamicsRegressor::configure error: link " << subtree_leaf_links[i] << " passed as a subtree leaf, but no FT sensor is attached to it " << std::endl; }
            return -2;
        }

        /** \todo add support to multiple FT sensors on the same link */
        if( nrOfFTSensorsAttachedToLink > 1 )
        {
            if( verbose ) { std::cerr << "subtreeBaseDynamicsRegressor::configure error: link " << subtree_leaf_links[i] << " passed as a subtree leaf, but more than one FT sensor is attached to it: this is not currenly implemented " << std::endl; }
            return -4;
        }

        /** \todo add further check that the indicated subtree is a proper one */
        subtree_leaf_links_indeces[i] = link_it->getLinkIndex();
    }

    //Now store all the links that belong to the subtree
    //We first compute the spanning starting at one (the first) of the leafs
    KDL::CoDyCo::Traversal subtree_traversal;

    undirected_tree.compute_traversal(subtree_traversal,subtree_leaf_links_indeces[0]);

    //Then we color the nodes: if white (true) the links belongs to the subtree
    //if black (false) it does not belongs to the node
    std::vector<bool> is_link_in_subtree(undirected_tree.getNrOfLinks(),false);

    //the leaf that we chose as starting link clearly belongs to the subtree
    is_link_in_subtree[subtree_leaf_links_indeces[0]] = true;

    //then, all the other links can be classified using the following rules:
    for(int i=1; i < (int)subtree_traversal.getNrOfVisitedLinks(); i++ ) {
        int link_id = subtree_traversal.getOrderedLink(i)->getLinkIndex();
        int parent_id = subtree_traversal.getParentLink(link_id)->getLinkIndex();

        if( is_link_in_subtree[parent_id] == false ) {
            //if the parent is not in the subtree (false/black) then the link is not in the subtree (false/black)
            is_link_in_subtree[link_id] = false;
        } else {
            int junction_index = undirected_tree.getLink(link_id)->getAdjacentJoint(undirected_tree.getLink(parent_id))->getJunctionIndex();

            int ft_sensor_index = getFTIndexFromJunctionIndex(sensors_tree, junction_index);
            if( ft_sensor_index >= 0 && isSubtreeLeaf(parent_id) ) {
                //if the parent is in the subtree (true/white) but it is connected to the link by an FT sensor (and the parent is a leaf of the subtree) then the link is not in the subtree (false/black)
                is_link_in_subtree[link_id] = false;
            } else {
                //otherwise the link is in the subtree (true/white)
                is_link_in_subtree[link_id] = true;
            }
        }
    }

    //after all the links belonging to the subtree have been identified, it is possible to save only the id of the one in the subtree
    assert( subtree_links_indices.size() == 0 );
    for( int i=0; i < (int)is_link_in_subtree.size(); i++ ) {
        if( is_link_in_subtree[i] ) {
            subtree_links_indices.push_back(i);
        }
    }

    //Configuring the relative_junctions vector
    relative_junctions.resize(0);

    for(int leaf_id = 0; leaf_id < (int) subtree_leaf_links_indeces.size(); leaf_id++ ) {
        if( consider_ft_offset ) {
            int leaf_link_id = subtree_leaf_links_indeces[leaf_id];

           int ft_sensor_index = getFirstFTSensorOnLink(sensors_tree,leaf_link_id);

            if( ft_sensor_index < 0 )
            {
                std::cerr << "[ERROR] subtreeBaseDynamicsRegressor: "
                          << " link "  << undirected_tree.getLink(leaf_link_id)->getName()
                          << " was indicated as leaf link of the subtree, but no FT sensor"
                          << " was found attached to this link" << std::endl;
                return false;
            }

            relative_junctions.push_back(ft_sensor_index);

        }
    }

    //Configuring the plucker frame in which the regressor is reported:
    // If the are no ft sensors of the subtree, project it in base frame
    //          (like for baseDynamicsRegressor)
    // If the are subtree leafs, project it on the first ft sensor plucker frame
    // Long term plan is to allow the user to select the frame in which project the reference frame
    if( sensors_tree.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE) == 0 )
    {
        regressor_plucker_frame = USE_DYNAMIC_BASE_PLUCKER_FRAME;
    }
    else
    {
        regressor_plucker_frame = USE_FIRST_FT_SENSOR_PLUCKER_FRAME;

        int first_leaf_link_id = subtree_leaf_links_indeces[0];
        int ft_id = getFirstFTSensorOnLink(*p_sensors_tree,first_leaf_link_id);

        iDynTree::SixAxisForceTorqueSensor * sens
                = (iDynTree::SixAxisForceTorqueSensor *) p_sensors_tree->getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft_id);

        first_ft_sensor_parent_link_id =
            undirected_tree.getJunction(sens->getParentIndex())->getParentLink()->getLinkIndex();

        assert( sens->isLinkAttachedToSensor(first_ft_sensor_parent_link_id) );

        iDynTree::Transform parent_link_H_sensor;

        sens->getLinkSensorTransform(first_ft_sensor_parent_link_id,parent_link_H_sensor);

        sensor_H_parent_link = parent_link_H_sensor.inverse();
    }

    iDynTree::Regressors::DynamicsRegressorParametersList localSerialization = getLegacyUsedParameters(linkIndeces2regrCols,
                                p_sensors_tree->getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE),
                                this->consider_ft_offset);

    regressor_local_parametrization.resize(6,localSerialization.parameters.size());

    return 0;
}

int  subtreeBaseDynamicsRegressor::getNrOfOutputs()
{
    return 6;
}

std::vector<int> subtreeBaseDynamicsRegressor::getRelativeJunctions()
{
    return relative_junctions;
}

iDynTree::Regressors::DynamicsRegressorParametersList subtreeBaseDynamicsRegressor::getUsedParameters()
{
    iDynTree::Regressors::DynamicsRegressorParametersList ret_values;

    assert(p_undirected_tree->getNrOfLinks() == linkIndeces2regrCols.size());

    // add considered links
    for(int i =0; i < (int)subtree_links_indices.size(); i++ )
    {
        int link_id = subtree_links_indices[i];
        // if a link is valid and not a fake link, push
        // in the vector of used parameters all the 10 link inertial parameters
        if( linkIndeces2regrCols[link_id] != -1 )
        {
            for(unsigned int link_param_type = 0 ; link_param_type < 10; link_param_type++ )
            {
                iDynTree::Regressors::DynamicsRegressorParameter param;
                param.category = iDynTree::Regressors::LINK_PARAM;
                param.elemIndex = link_id;
                param.type = getLinkParameterType(link_param_type);
                ret_values.addParam(param);
            }
        }
    }


    for(int leaf_id = 0; leaf_id < (int) subtree_leaf_links_indeces.size(); leaf_id++ )
    {
        if( consider_ft_offset )
        {
            int leaf_link_id = subtree_leaf_links_indeces[leaf_id];

            int ft_id = getFirstFTSensorOnLink(*p_sensors_tree,leaf_link_id);
            for(unsigned int ft_param_type = 0 ; ft_param_type < 6; ft_param_type++ )
            {
                iDynTree::Regressors::DynamicsRegressorParameter param;
                param.category = iDynTree::Regressors::SENSOR_FT_PARAM;
                param.elemIndex = ft_id;
                param.type = getFTParameterType(ft_param_type);
                ret_values.addParam(param);
            }
        }
    }

    return ret_values;
}



int  subtreeBaseDynamicsRegressor::computeRegressor(const KDL::JntArray &/*q*/,
                                                    const KDL::JntArray &/*q_dot*/,
                                                    const KDL::JntArray &/*q_dotdot*/,
                                                    const std::vector<KDL::Frame> & X_dynamic_base,
                                                    const std::vector<KDL::Twist> & v,
                                                    const std::vector<KDL::Twist> & a,
                                                    const iDynTree::SensorsMeasurements & sensor_measurements,
                                                    const KDL::JntArray &/*measured_torques*/,
                                                    Eigen::MatrixXd & regressor_matrix_global_column_serialization,
                                                    Eigen::VectorXd & known_terms)
{
    if( this->regressor_local_parametrization.rows() != 6 ) {
        return -1;
    }

    //We have to project all the regressor in a common frame
    KDL::Frame X_measure_dynamic_base;

    switch( regressor_plucker_frame )
    {
        case USE_DYNAMIC_BASE_PLUCKER_FRAME:
            X_measure_dynamic_base = KDL::Frame::Identity();
        break;
        case USE_FIRST_FT_SENSOR_PLUCKER_FRAME:
            X_measure_dynamic_base = iDynTree::ToKDL(sensor_H_parent_link)*(X_dynamic_base[first_ft_sensor_parent_link_id].Inverse());
        break;
    }


    //all other columns, beside the one relative to the inertial parameters of the links of the subtree, are zero
    regressor_local_parametrization.setZero();


    for(int i =0; i < (int)subtree_links_indices.size(); i++ ) {
        int link_id = subtree_links_indices[i];

        if( linkIndeces2regrCols[link_id] != -1 ) {

            Eigen::Matrix<double,6,10> netWrenchRegressor_i = netWrenchRegressor(v[link_id],a[link_id]);


            regressor_local_parametrization.block(0,(int)(10*linkIndeces2regrCols[link_id]),6,10) = WrenchTransformationMatrix(X_measure_dynamic_base*X_dynamic_base[link_id])*netWrenchRegressor_i;
        }
    }

    //if the ft offset is condidered, we have to set also the columns relative to the offset
    //For each subgraph, we consider the measured wrenches as the one excerted from the remain of the tree to the considered subtree
    //So the sign of the offset should be consistent with the other place where it is defined. In particular, the offset of a sensor is considered
    //as an addictive on the measured wrench, so f_s = f_sr + f_o, where the frame of expression and the sign of f_s is defined in
    //KDL::CoDyCo::FTSensor
    for(int leaf_id = 0; leaf_id < (int) subtree_leaf_links_indeces.size(); leaf_id++ ) {
        if( consider_ft_offset ) {
            int leaf_link_id = subtree_leaf_links_indeces[leaf_id];

            int ft_id = getFirstFTSensorOnLink(*p_sensors_tree,leaf_link_id);

            assert(ft_id >= 0);

            iDynTree::SixAxisForceTorqueSensor * sens
                = (iDynTree::SixAxisForceTorqueSensor *) p_sensors_tree->getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft_id);

            assert( sens->isLinkAttachedToSensor(leaf_link_id) );

            /** \todo find a more robust way to get columns indeces relative to a given parameters */
            assert(ft_id >= 0 && ft_id < 100);
            if( !(10*NrOfRealLinks_subtree+6*ft_id+5 < regressor_local_parametrization.cols()) ) {
                std::cout << "NrOfRealLinks " << NrOfRealLinks_subtree << std::endl;
                std::cout << "ft_id         " << ft_id << std::endl;
            }
            assert(10*NrOfRealLinks_subtree+6*ft_id+5 < regressor_local_parametrization.cols());

            double sign;
            if( sens->getAppliedWrenchLink() == leaf_link_id ) {
                sign = 1.0;
            } else {
                sign = -1.0;
            }

            iDynTree::Transform leaf_link_H_sensor;

            bool ok = sens->getLinkSensorTransform(leaf_link_id,leaf_link_H_sensor);

            assert(ok);

            regressor_local_parametrization.block(0,(int)(10*NrOfRealLinks_subtree+6*ft_id),6,6)
                = sign*WrenchTransformationMatrix(X_measure_dynamic_base*X_dynamic_base[leaf_link_id]*iDynTree::ToKDL(leaf_link_H_sensor));

        }
    }

    //The known terms are simply all the measured wrenches acting on the subtree
    known_terms.setZero();

    #ifndef NDEBUG
    //std::cerr << "computing kt " << std::endl;
    //std::cerr << (ft_list).toString();
#endif
    for(int leaf_id = 0; leaf_id < (int) subtree_leaf_links_indeces.size(); leaf_id++ ) {
        int leaf_link_id = subtree_leaf_links_indeces[leaf_id];

        int ft_id = getFirstFTSensorOnLink(*p_sensors_tree,leaf_link_id);

        assert(ft_id >= 0);

        iDynTree::SixAxisForceTorqueSensor * sens
            = (iDynTree::SixAxisForceTorqueSensor *) p_sensors_tree->getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft_id);

        assert( sens->isLinkAttachedToSensor(leaf_link_id) );

        iDynTree::Wrench sensor_wrench;
        iDynTree::Wrench wrench_on_link_measured;

        bool ok = sensor_measurements.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft_id,sensor_wrench);

        ok = ok && sens->getWrenchAppliedOnLink(leaf_link_id,sensor_wrench,wrench_on_link_measured);

        assert(ok);

        known_terms += toEigen(X_measure_dynamic_base*(X_dynamic_base[leaf_link_id]*iDynTree::ToKDL(wrench_on_link_measured)));
    }

#ifndef NDEBUG
/*
std::cout << "Returning from computeRegressor (subtree):" << std::endl;
std::cout << "Regressor " << std::endl;
std::cout << regressor_matrix << std::endl;
std::cout << "known terms" << std::endl;
std::cout << known_terms << std::endl;
*/
#endif

    convertLocalRegressorToGlobalRegressor(regressor_local_parametrization,regressor_matrix_global_column_serialization,this->localParametersIndexToOutputParametersIndex);

    return 0;
}

bool subtreeBaseDynamicsRegressor::setGlobalParameters(const iDynTree::Regressors::DynamicsRegressorParametersList& globalParameters)
{
    iDynTree::Regressors::DynamicsRegressorParametersList localSerialiaziation =
        getLegacyUsedParameters(linkIndeces2regrCols,
                                p_sensors_tree->getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE),
                                this->consider_ft_offset);

    buildParametersMapping(localSerialiaziation,globalParameters,this->localParametersIndexToOutputParametersIndex);

    return true;
}



}

}
}
