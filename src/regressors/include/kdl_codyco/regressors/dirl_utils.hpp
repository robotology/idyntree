/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 */

#ifndef _DIRL_UTILS_
#define _DIRL_UTILS_

#include <Eigen/Dense>

#include "kdl_codyco/undirectedtree.hpp"

#include "iDynTree/Sensors/Sensors.hpp"
#include "iDynTree/Sensors/SixAxisFTSensor.hpp"
#include "iDynTree/Core/Wrench.h"
#include "iDynTree/Regressors/DynamicsRegressorParameters.h"

namespace KDL {
namespace CoDyCo {
namespace Regressors
{
    /**
     * Measure of sparsity of a matrix, equal to the ratio of the zero elements on the total elements
     * @return 1.0 for a matrix made only of zeros, 0.0 for a completly dense matrix
     */
    double sparsity_index(const Eigen::MatrixXd & mat, const double tol);


    /**
     * Return a matrix where all the elements near to zero ar set to 0.0
     *
     */
    Eigen::MatrixXd zeroToZero(const Eigen::MatrixXd & input_mat, double tol=1e-5);

   /**
    * @param input_matrix a n x m matrix
    * @param row_space_basis_matrix a m X rank matrix, whose columns form a base for the row space of input_matrix
    * @param tol (optional) tollerance to use for calculating the rank of the matrix (default: max(n,m)*max(sigma)*machine_epslion)
    *
    * \note This function allocate dynamically memory, so it is not real time safe
    */
    int getRowSpaceBasis(const Eigen::MatrixXd & input_matrix, Eigen::MatrixXd & row_space_basis_matrix, double tol = -1.0, bool verbose = false );
    int getRowSpaceBasis(const Eigen::MatrixXd & input_matrix, Eigen::MatrixXd & row_space_basis_matrix, double tol, bool verbose, Eigen::VectorXd & sigma);

    /**
     * Calculate the intersection of two given subspaces using Golub Principal Angles Algorithm from section 12.4.4 of Golub - Van Lohan Matrix Computation book
     *
     *
     */
    int getSubSpaceIntersection(const Eigen::MatrixXd & first_subspace, const Eigen::MatrixXd & second_subspace, Eigen::MatrixXd & result, double tol=-1.0, bool verbose=false);

    int getKernelSpaceBasis(const Eigen::MatrixXd & input_matrix, Eigen::MatrixXd & row_space_basis_matrix, double tol = -1.0, bool verbose = false);

    /**
     * Return the index of the first six axis FT sensor attached to a given link.
     *
     * @return the index of the first FT sensor attached to the link,
     *         or -1 in case no ft sensor is attached to the link
     */
    int getFirstFTSensorOnLink(const iDynTree::SensorsList & sensors_tree,
                               const int link_id);

    /**
     * Return the number of six axis FT sensors attached to a given link.
     *
     */
    int getNrOfFTSensorsOnLink(const iDynTree::SensorsList & sensors_tree,
                               const int link_id);

    /**
     * Return the sensor index of the FT sensor associated to a junction.
     *
     * @return the index of the FT sensor associated to the junction, or -1
     *         if not FT sensor is associated to the junction.
     *
     */
     int getFTIndexFromJunctionIndex(const  iDynTree::SensorsList & sensors_tree,
                                     const int junction_id);
            // For the time being, simulate the sensor measurement from the robot
       // state using the low-level datastructure representing internal forces
       // In the long term, we should have a KDL::CoDyCo::RobotDynamicState class
       // representing the redundant state (position,velocities,acceleration,internal & external forces)
       // this will permit to have for all sensor a similar signature:
//        bool simulateMeasurement(const KDL::CoDyCo::RobotDynamicState & state,
//                                     MeasurementType & simulated_measurement);

     bool simulateMeasurement_sixAxisFTSensor(KDL::CoDyCo::Traversal & dynamic_traversal,
                              std::vector<KDL::Wrench> f,
                              iDynTree::SixAxisForceTorqueSensor *sixAxisForceTorqueSensor,
                              iDynTree::Wrench & simulated_measurement);

     /**
      * For each number from 0 to 9, get a distint link related DynamicsRegressorParameterType.
      *
      */
     iDynTree::Regressors::DynamicsRegressorParameterType getLinkParameterType(unsigned int nr);

     /**
      * For each number from 0 to 5, get a distint ft related DynamicsRegressorParameterType.
      *
      */
     iDynTree::Regressors::DynamicsRegressorParameterType getFTParameterType(unsigned int nr);

     /**
      * Encoded serialization of inertial parameters used
      * Return a number from 0 to 9 given an inertial parameter type.
      * The serialization used is:
      *   m mx my mz ixx ixy ixz iyy iyz izz
      *
      */
     int getInertialParameterLocalIndex(const iDynTree::Regressors::DynamicsRegressorParameterType & type);



    /**
     * Get the legacy list of parameters that is currently hardcoded in all the subregressors.
     *
     */
    iDynTree::Regressors::DynamicsRegressorParametersList
        getLegacyUsedParameters(const std::vector<int> & linkIndeces2regrCols,
                                const int nrOfFTSensors = -1,
                                const bool withFToffsetParameters=false);

    /**
     *
     * Copy a local regressor (with columns following the localSerialization list)
     * in a global one (with the columns following the globalSerialiation list).
     *
     */
     void convertLocalRegressorToGlobalRegressor(const Eigen::MatrixXd & localRegressor,
                                            Eigen::MatrixXd & globalRegressor,
                                            std::vector<int> localColIndecesToGlobalColIndeces);

     void convertLocalParametersToGlobalParameters(const Eigen::VectorXd & localRegressor,
                                              Eigen::VectorXd & globalRegressor,
                                              std::vector<int> localColIndecesToGlobalColIndeces);


    void buildParametersMapping(const iDynTree::Regressors::DynamicsRegressorParametersList & localSerialiaziation,
                                const iDynTree::Regressors::DynamicsRegressorParametersList & globalSerialiaziation,
                                std::vector<int> & localParametersIndexToOutputParametersIndex);



}

}

}

#endif
