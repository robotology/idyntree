/**
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 */

#include "dirl_utils.hpp"

#include "kdl_codyco/regressor_utils.hpp"

#include "iDynTree/Sensors/SixAxisFTSensor.h"
#include "iDynTree/Core/Transform.h"
#include "kdl_codyco/KDLConversions.h"
#include <iostream>

#include <cfloat>


namespace KDL {
namespace CoDyCo {
namespace Regressors {


double sparsity_index(const Eigen::MatrixXd & mat, const double tol)
    {
        int zero_elements = 0;
        for(int i=0; i < mat.rows(); i++ ) {
            for(int j=0; j < mat.cols(); j++ ) {
                if( fabs(mat(i,j)) < tol ) {
                    zero_elements++;
                }
            }
        }
        return ((double)zero_elements)/(mat.rows()*mat.cols());
    }

Eigen::MatrixXd zeroToZero(const Eigen::MatrixXd & input_mat, double tol)
{
    Eigen::MatrixXd output_mat;
    output_mat = input_mat;

    for(int i=0; i < output_mat.rows(); i++ ) {
        for(int j=0; j < output_mat.cols(); j++ ) {
            if( fabs(output_mat(i,j)) < tol ) {
                output_mat(i,j) = 0.0;
            }
        }
    }
    return output_mat;
}

int getKernelSpaceBasis(const Eigen::MatrixXd & input_matrix, Eigen::MatrixXd & kernel_basis_matrix, double tol, bool verbose)
{
    if( input_matrix.rows() == 0 ) {
        kernel_basis_matrix = Eigen::MatrixXd::Identity(input_matrix.cols(),input_matrix.cols());
        return 0;
    }
        //std::cout << "Called getRowSpaceBasis with input_matrix: " << std::endl;
        //std::cout << input_matrix << std::endl;
        //Probably can be improved using a different decomposition (QR?)
        //NOT REAL TIME!!
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(input_matrix, Eigen::ComputeThinU | Eigen::ComputeFullV);



        int n = input_matrix.rows();
        int m = input_matrix.cols();

        Eigen::VectorXd sigma = svd.singularValues();

        if(verbose) {  std::cout << "Singular values " << std::endl; std::cout << sigma << std::endl; }

        if( tol <= 0 ) {
            /** \todo find a better and consistend heuristic */
            //To avoid problem on numerically zero matrices
            if( sigma[0] >= sqrt(DBL_EPSILON) ) {
                tol = 1000*sigma[0]*std::max(n,m)*DBL_EPSILON;
            } else {
                //Matrix is probably numerically zero
                //It is wise to consider all the matrix as a zero matrix
                tol = sqrt(DBL_EPSILON);
            }
        }

        int ll;
        for(ll=0; ll < sigma.size(); ll++ ) {
            if( sigma[ll] < tol ) { break;}
        }

        int rank = ll;

        Eigen::MatrixXd V = svd.matrixV();
        assert(V.cols() == V.rows());
        if( V.cols() != V.rows() ) { std::cout << "V is not square" << std::endl; }
        assert(rank <= m);

        kernel_basis_matrix.resize(m,m-rank);
        std::cout << "Matrix has rank " << rank << std::endl; std::cout << " m " << m << " rank " << rank << " V size " << V.rows() << " " << V.cols() << std::endl;
        kernel_basis_matrix = V.block(0,rank,m,m-rank);
        std::cout << "basis calculated, tol used " << tol << std::endl;

        return 0;
}

int getSubSpaceIntersection(const Eigen::MatrixXd & first_subspace, const Eigen::MatrixXd & second_subspace, Eigen::MatrixXd & result, double tol, bool /*verbose*/)
{
    std::cout << "getSubSpaceIntersection" << std::endl;

    std::cout << "first_subspace " << first_subspace.rows() << " " << first_subspace.cols() << std::endl;
    //std::cout << first_subspace << std::endl;
    std::cout << "second_subspace " << second_subspace.rows() << " " << second_subspace.cols() << std::endl;
    //std::cout << second_subspace << std::endl;

    if( tol <= 0.0 ) { tol = 1e-7; }
    //The input matrices columns in input should form a basis of a subspace of a common vector space
    if( first_subspace.rows() != second_subspace.rows() ) { return -1; }
    if( first_subspace.cols() > first_subspace.rows() ) { return -1; }
    if( second_subspace.cols() > second_subspace.rows() ) { return -1; }
    if( first_subspace.cols() == 0 || second_subspace.cols() == 0 ) { result.resize(first_subspace.rows(),0); return 0; }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(first_subspace.transpose()*second_subspace, Eigen::ComputeFullU | Eigen::ComputeThinV);

    Eigen::VectorXd sigma = svd.singularValues();
    //std::cout << "Sigma " << std::endl << sigma << std::endl;
    int intersection_size = 0;
    for(int i=0; i < sigma.size(); i++ ) {
        if( fabs(sigma[i]-1) < tol ) {
            intersection_size++;
        } else {
            break;
        }
    }

    std::cout << "intersection_size " << intersection_size << std::endl;


    if( intersection_size == 0 ) { result.resize(first_subspace.rows(),0); return 0; }

    //result.resize(first_subspace.rows(),intersection_size);
    //std::cout << "Before compute" << std::endl;
    //std::cout << "After compute" << std::endl;
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::MatrixXd U = svd.matrixU();
    //std::cout << "After assignment" << std::endl;
    //std::cout << "Second subspace size: " << second_subspace.rows() << " " << second_subspace.cols() << std::endl;
    //std::cout << "V size: " << V.rows() << " " << V.cols() << std::endl;
    //std::cout << "intersection_size " << intersection_size << std::endl;

    for(int i=0; i < intersection_size; i++ ) {
        //result = (second_subspace*V.transpose()).block(0,0,second_subspace.rows(),intersection_size);
        result = (first_subspace*U.transpose().block(0,0,U.cols(),intersection_size));

    }

    return 0;
}

int getRowSpaceBasis(const Eigen::MatrixXd & input_matrix, Eigen::MatrixXd & row_space_basis_matrix, double tol, bool verbose)
{
    Eigen::VectorXd dummy;
    return getRowSpaceBasis(input_matrix,row_space_basis_matrix,tol,verbose,dummy);
}

int getRowSpaceBasis(const Eigen::MatrixXd & input_matrix, Eigen::MatrixXd & row_space_basis_matrix, double tol, bool /*verbose*/, Eigen::VectorXd & sigma)
{
    if( input_matrix.rows() == 0 ) {
        row_space_basis_matrix.resize(input_matrix.cols(),0);
        return 0;
    }
        std::cout << "Called getRowSpaceBasis " << std::endl;
        //std::cout << input_matrix << std::endl;
        //Probably can be improved using a different decomposition (QR?)
        //NOT REAL TIME!!
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(input_matrix, Eigen::ComputeThinU | Eigen::ComputeFullV);



        int n = input_matrix.rows();
        int m = input_matrix.cols();

        sigma = svd.singularValues();


        if( tol <= 0 ) {
            /** \todo find a better and consistend heuristic */
            //To avoid problem on numerically zero matrices
            if( sigma[0] >= sqrt(DBL_EPSILON) ) {
                tol = 1000*sigma[0]*std::max(n,m)*DBL_EPSILON;
            } else {
                //Matrix is probably numerically zero
                //It is wise to consider all the matrix as a zero matrix
                tol = sqrt(DBL_EPSILON);
            }
        }

        int ll;
        for(ll=0; ll < sigma.size(); ll++ ) {
            if( sigma[ll] < tol ) { break;}
        }

        int rank = ll;

        Eigen::MatrixXd V = svd.matrixV();
        assert(V.cols() == V.rows());
        if( V.cols() != V.rows() ) { std::cout << "V is not square" << std::endl; }
        assert(rank <= m);

        row_space_basis_matrix.resize(m,rank);
        std::cout << "Matrix has rank " << rank << std::endl; std::cout << " m " << m << " rank " << rank << " V size " << V.rows() << " " << V.cols() << std::endl;
        row_space_basis_matrix = V.block(0,0,m,rank);
        std::cout << "basis calculated, tol used " << tol << std::endl;

        return 0;
}

int getFirstFTSensorOnLink(const iDynTree::SensorsList & sensors_tree,
                           const int link_id)
{
    for(int ft=0; ft < sensors_tree.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); ft++ )
    {
        iDynTree::SixAxisForceTorqueSensor * sens
            = (iDynTree::SixAxisForceTorqueSensor *) sensors_tree.getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft);

        assert(sens != 0);

        if( sens->isLinkAttachedToSensor(link_id) )
        {
            return ft;
        }

    }

    return -1;
}

int getNrOfFTSensorsOnLink(const iDynTree::SensorsList & sensors_tree,
                           const int link_id)
{
    int nrOfFTSensorsOnLink = 0;
    for(int ft=0; ft < sensors_tree.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); ft++ )
    {
        iDynTree::SixAxisForceTorqueSensor * sens
            = (iDynTree::SixAxisForceTorqueSensor *) sensors_tree.getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft);

        assert(sens != 0);

        if( sens->isLinkAttachedToSensor(link_id) )
        {
            nrOfFTSensorsOnLink = nrOfFTSensorsOnLink + 1;
        }

    }

    return nrOfFTSensorsOnLink;
}

int getFTIndexFromJunctionIndex(const iDynTree::SensorsList & sensors_tree,
                                const int junction_id)
{
    for(int ft=0; ft < sensors_tree.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); ft++ )
    {
        iDynTree::SixAxisForceTorqueSensor * sens
            = (iDynTree::SixAxisForceTorqueSensor *) sensors_tree.getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft);

        assert(sens != 0);

        if( sens->getParentJointIndex() == junction_id )
        {
            return ft;
        }

    }

    return -1;

}

bool simulateMeasurement_sixAxisFTSensor(KDL::CoDyCo::Traversal & dynamic_traversal,
                                                   std::vector< KDL::Wrench > f,
                                                   iDynTree::SixAxisForceTorqueSensor *sixAxisForceTorqueSensor,
                                                   iDynTree::Wrench& simulated_measurement)
{
    //Check that the input size is consistent
    assert(f.size() == dynamic_traversal.getNrOfVisitedLinks());
    assert(sixAxisForceTorqueSensor->isValid());
    assert(sixAxisForceTorqueSensor->getFirstLinkIndex() > 0 && sixAxisForceTorqueSensor->getFirstLinkIndex() < dynamic_traversal.getNrOfVisitedLinks());
    assert(sixAxisForceTorqueSensor->getSecondLinkIndex() > 0 && sixAxisForceTorqueSensor->getSecondLinkIndex() < dynamic_traversal.getNrOfVisitedLinks());


    // The f vector is assume to be the output of the rneaDynamicLoop function,
    // ie f[i] is the force applied by link i on the link dynamic_traversal.getParent(i),
    // expressed in the refernce frame of link i
    // From this information, we can "simulate" the output that we could expect on this sensor

    // First we get the two links attached to this ft sensor, and we check which one is the
    // parent and which one is the child in the dynamic_traversal Traversal
    int child_link = -1;
    int parent_link = -1;
    if( dynamic_traversal.getParentLink(sixAxisForceTorqueSensor->getFirstLinkIndex())->getLinkIndex() == sixAxisForceTorqueSensor->getSecondLinkIndex() )
    {
        child_link = sixAxisForceTorqueSensor->getFirstLinkIndex();
        parent_link = sixAxisForceTorqueSensor->getSecondLinkIndex();
    }
    else
    {
        assert(dynamic_traversal.getParentLink(sixAxisForceTorqueSensor->getSecondLinkIndex())->getLinkIndex() == sixAxisForceTorqueSensor->getFirstLinkIndex() );
        child_link = sixAxisForceTorqueSensor->getSecondLinkIndex();
        parent_link = sixAxisForceTorqueSensor->getFirstLinkIndex();
    }

    // if the child_link is the link to which the measured wrench is applied, the sign between the
    // measured_wrench and f[child_link] is consistent, otherwise we have to change the sign



    // To simulate the sensor, we have to translate f[child] in the sensor frame
    // with the appriopriate sign
    iDynTree::Transform child_link_H_sensor;
    sixAxisForceTorqueSensor->getLinkSensorTransform(child_link,child_link_H_sensor);
    if( sixAxisForceTorqueSensor->getAppliedWrenchLink() == parent_link  )
    {
        simulated_measurement = -(child_link_H_sensor.inverse()*iDynTree::ToiDynTree(f [child_link]));
    }
    else
    {
        simulated_measurement = (child_link_H_sensor.inverse()*iDynTree::ToiDynTree(f[child_link]));
        assert( sixAxisForceTorqueSensor->getAppliedWrenchLink() == child_link );

    }

    return true;
}

iDynTree::Regressors::DynamicsRegressorParameterType getLinkParameterType(unsigned int nr)
{
    assert(nr >= 0 && nr < 10);
    switch(nr)
    {
        case 0:
            return iDynTree::Regressors::LINK_MASS;
        case 1:
            return iDynTree::Regressors::LINK_FIRST_MOMENT_OF_MASS_X;
        case 2:
            return iDynTree::Regressors::LINK_FIRST_MOMENT_OF_MASS_Y;
        case 3:
            return iDynTree::Regressors::LINK_FIRST_MOMENT_OF_MASS_Z;
        case 4:
            return iDynTree::Regressors::LINK_MOMENT_OF_INERTIA_XX;
        case 5:
            return iDynTree::Regressors::LINK_MOMENT_OF_INERTIA_XY;
        case 6:
            return iDynTree::Regressors::LINK_MOMENT_OF_INERTIA_XZ;
        case 7:
            return iDynTree::Regressors::LINK_MOMENT_OF_INERTIA_YY;
        case 8:
            return iDynTree::Regressors::LINK_MOMENT_OF_INERTIA_YZ;
        case 9:
            return iDynTree::Regressors::LINK_MOMENT_OF_INERTIA_ZZ;
    }
}

iDynTree::Regressors::DynamicsRegressorParameterType getFTParameterType(unsigned int nr)
{
    assert(nr >= 0 && nr < 6);
    switch(nr)
    {
        case 0:
            return iDynTree::Regressors::SENSOR_FT_OFFSET_FORCE_X;
        case 1:
            return iDynTree::Regressors::SENSOR_FT_OFFSET_FORCE_Y;
        case 2:
            return iDynTree::Regressors::SENSOR_FT_OFFSET_FORCE_Z;
        case 3:
            return iDynTree::Regressors::SENSOR_FT_OFFSET_TORQUE_X;
        case 4:
            return iDynTree::Regressors::SENSOR_FT_OFFSET_TORQUE_Y;
        case 5:
            return iDynTree::Regressors::SENSOR_FT_OFFSET_TORQUE_Z;
    }
}

int getInertialParameterLocalIndex(const iDynTree::Regressors::DynamicsRegressorParameterType & type)
{
    int ret_val;

    switch(type)
    {
        case iDynTree::Regressors::LINK_MASS:
            ret_val = 0;
            break;
        case iDynTree::Regressors::LINK_FIRST_MOMENT_OF_MASS_X:
            ret_val = 1;
            break;
        case iDynTree::Regressors::LINK_FIRST_MOMENT_OF_MASS_Y:
            ret_val = 2;
            break;
        case iDynTree::Regressors::LINK_FIRST_MOMENT_OF_MASS_Z:
            ret_val = 3;
            break;
        case iDynTree::Regressors::LINK_MOMENT_OF_INERTIA_XX:
            ret_val = 4;
            break;
        case iDynTree::Regressors::LINK_MOMENT_OF_INERTIA_XY:
            ret_val = 5;
            break;
        case iDynTree::Regressors::LINK_MOMENT_OF_INERTIA_XZ:
            ret_val = 6;
            break;
        case iDynTree::Regressors::LINK_MOMENT_OF_INERTIA_YY:
            ret_val = 7;
            break;
        case iDynTree::Regressors::LINK_MOMENT_OF_INERTIA_YZ:
            ret_val = 8;
            break;
        case iDynTree::Regressors::LINK_MOMENT_OF_INERTIA_ZZ:
            ret_val = 9;
            break;
        default:
            ret_val = -1;
            break;
    }
    return ret_val;
}




iDynTree::Regressors::DynamicsRegressorParametersList
    getLegacyUsedParameters(const std::vector<int> & linkIndeces2regrCols,
                            const int nrOfFTSensors,
                            const bool withFToffset )
{
    iDynTree::Regressors::DynamicsRegressorParametersList ret_values;

    // add considered links
    for(unsigned int link=0; link < linkIndeces2regrCols.size(); link++ )
    {
        // if a link is valid and not a fake link, push
        // in the vector of used parameters all the 10 link inertial parameters
        if( linkIndeces2regrCols[link] != -1 )
        {
            for(unsigned int link_param_type = 0 ; link_param_type < 10; link_param_type++ )
            {
                iDynTree::Regressors::DynamicsRegressorParameter param;
                param.category = iDynTree::Regressors::LINK_PARAM;
                param.elemIndex = link;
                param.type = getLinkParameterType(link_param_type);
                ret_values.addParam(param);
            }
        }
    }

    if( withFToffset )
    {
        assert(nrOfFTSensors >= 0);
        for(unsigned int ft = 0; ft < (unsigned int)nrOfFTSensors; ft++ )
        {
            for(unsigned int ft_param_type = 0 ; ft_param_type < 6; ft_param_type++ )
            {
                iDynTree::Regressors::DynamicsRegressorParameter param;
                param.category = iDynTree::Regressors::SENSOR_FT_PARAM;
                param.elemIndex = ft;
                param.type = getFTParameterType(ft_param_type);
                ret_values.addParam(param);
            }
        }
    }

    return ret_values;
}


void convertLocalRegressorToGlobalRegressor(const Eigen::MatrixXd & localRegressor,
                                            Eigen::MatrixXd & globalRegressor,
                                            std::vector<int> localColIndecesToGlobalColIndeces)
{
    assert(localRegressor.rows() == globalRegressor.rows());
    assert(localRegressor.cols() == localColIndecesToGlobalColIndeces.size());

    for(int localParam = 0; localParam <  (int)localColIndecesToGlobalColIndeces.size(); localParam++)
    {
        int globalParam = localColIndecesToGlobalColIndeces[localParam];

        if( globalParam >= 0 )
        {
            globalRegressor.col(globalParam) = localRegressor.col(localParam);
        }
    }

    return;
}

void convertLocalParametersToGlobalParameters(const Eigen::VectorXd & localRegressor,
                                              Eigen::VectorXd & globalRegressor,
                                              std::vector<int> localColIndecesToGlobalColIndeces)
{
    assert(localRegressor.rows() == globalRegressor.rows());
    assert(localRegressor.cols() == localColIndecesToGlobalColIndeces.size());

    for(int localParam = 0; localParam <  (int)localColIndecesToGlobalColIndeces.size(); localParam++)
    {
        int globalParam = localColIndecesToGlobalColIndeces[localParam];

        if( globalParam >= 0 )
        {
            globalRegressor(globalParam) = localRegressor(localParam);
        }
    }

    return;
}


void buildParametersMapping(const iDynTree::Regressors::DynamicsRegressorParametersList & localSerialiaziation,
                                const iDynTree::Regressors::DynamicsRegressorParametersList & globalSerialiaziation,
                                std::vector<int> & localParametersIndexToOutputParametersIndex)
{

    localParametersIndexToOutputParametersIndex.resize(localSerialiaziation.parameters.size());
    for(unsigned int localIndex = 0; localIndex < localParametersIndexToOutputParametersIndex.size(); localIndex++ )
    {
        iDynTree::Regressors::DynamicsRegressorParameter localParam;

        localParam = localSerialiaziation.parameters[localIndex];

        unsigned int globalIndex;
        bool localIsInGlobal = globalSerialiaziation.findParam(localParam,globalIndex);

        if( localIsInGlobal )
        {
            localParametersIndexToOutputParametersIndex[localIndex] = globalIndex;
        }
        else
        {
            localParametersIndexToOutputParametersIndex[localIndex] = -1;
        }
    }

    return;
}





}

}

}
