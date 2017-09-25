/*
* Copyright (C) 2015 RBCS Department, Istituto Italiano di Tecnologia
* Author: Silvio Traversaro <silvio.traversaro@iit.it>
*
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/
//

#include "testModels.h"

#include <kdl/tree.hpp>
#include <kdl_codyco/undirectedtree.hpp>
#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/SixAxisFTSensor.h>

#include <kdl_codyco/KDLConversions.h>

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Regressors/DynamicsRegressorGenerator.h>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Twist.h>

#include <iDynTree/ModelIO/impl/urdf_import.hpp>
#include <iDynTree/ModelIO/impl/urdf_sensor_import.hpp>

#include <kdl_codyco/regressors/dynamicRegressorGenerator.hpp>
#include <kdl_codyco/regressor_loops.hpp>
#include <kdl_codyco/rnea_loops.hpp>

#include <kdl_codyco/regressor_utils.hpp>
#include <kdl_codyco/regressors/dirl_utils.hpp>

#include <iostream>
#include <string>
#include <vector>

#include <cstdlib>
#include <cassert>

double random_double()
{
    return ((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

iDynTree::Wrench simulateFTSensorFromKinematicState(const KDL::CoDyCo::UndirectedTree & icub_undirected_tree,
                                        const KDL::JntArray & q,
                                        const KDL::JntArray & dq,
                                        const KDL::JntArray & ddq,
                                        const KDL::Twist    & base_velocity,
                                        const KDL::Twist    & base_acceleration,
                                        const std::string ft_sensor_name,
                                        const iDynTree::SensorsList & sensors_tree
                                        )
{
    // We can try to simulate the same sensor with the usual inverse dynamics
    KDL::CoDyCo::Traversal traversal;

    icub_undirected_tree.compute_traversal(traversal);

    std::vector<KDL::Twist> v(icub_undirected_tree.getNrOfLinks());
    std::vector<KDL::Twist> a(icub_undirected_tree.getNrOfLinks());
    std::vector<KDL::Wrench> f(icub_undirected_tree.getNrOfLinks());
    std::vector<KDL::Wrench> f_gi(icub_undirected_tree.getNrOfLinks());
    std::vector<KDL::Wrench> f_ext(icub_undirected_tree.getNrOfLinks(),KDL::Wrench::Zero());
    KDL::JntArray torques(icub_undirected_tree.getNrOfDOFs());
    KDL::Wrench base_force;

    KDL::CoDyCo::rneaKinematicLoop(icub_undirected_tree,
                                   q,dq,ddq,traversal,base_velocity,base_acceleration,
                                   v,a,f_gi);
    KDL::CoDyCo::rneaDynamicLoop(icub_undirected_tree,q,traversal,f_gi,f_ext,f,torques,base_force);

    unsigned int sensor_index;
    sensors_tree.getSensorIndex(iDynTree::SIX_AXIS_FORCE_TORQUE,ft_sensor_name,sensor_index);

    std::cout << ft_sensor_name << " has ft index " << sensor_index << std::endl;

    iDynTree::SixAxisForceTorqueSensor * p_sens
            = (iDynTree::SixAxisForceTorqueSensor *) sensors_tree.getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,sensor_index);

    iDynTree::Wrench simulate_measurement;

    KDL::CoDyCo::Regressors::simulateMeasurement_sixAxisFTSensor(traversal,f,p_sens,simulate_measurement);


    return simulate_measurement;
}

// This function logic should be moved to some better place for sure


int main()
{
    std::string icub_urdf_filename = getAbsModelPath("icub_model.urdf");

    // Create the iCub model
    KDL::Tree icub_tree;
    bool ok = iDynTree::treeFromUrdfFile(icub_urdf_filename,icub_tree);

    if( !ok )
    {
        std::cerr <<"Could not import icub urdf" << std::endl;
        return EXIT_FAILURE;
    }

    // Create the iCub undirected tree, that contains also
    // a default serialization (i.e. a mapping between links/joints
    // and integers (if you want to provide a user-defined serialization
    // to the undirected tree, pass it as a second argument to the constructor
    KDL::CoDyCo::UndirectedTree icub_undirected_tree(icub_tree);

    std::cout << "icub_tree serialization 1 : " << icub_undirected_tree.getSerialization().toString();

    // Load a sensors tree (for ft sensors) from the information extracted from urdf file
    //  and using the serialization provided in the undirected tree
    iDynTree::SensorsList sensors_tree = iDynTree::sensorsListFromURDF(icub_undirected_tree,icub_urdf_filename);

    //Create a regressor generator
    KDL::CoDyCo::Regressors::DynamicRegressorGenerator regressor_generator(icub_undirected_tree,sensors_tree);

    //Add a set of rows of the regressor of right arm
    std::vector<std::string> l_arm_subtree;

    l_arm_subtree.push_back("l_upper_arm");

    regressor_generator.addSubtreeRegressorRows(l_arm_subtree);

    //Create a random state for the robot
    KDL::Twist base_velocity, base_acceleration;
    base_velocity = base_acceleration = KDL::Twist::Zero();

    KDL::JntArray q(regressor_generator.getNrOfDOFs());

    SetToZero(q);

    srand(time(0));
    for(unsigned int i=0; i < q.rows(); i++ )
    {
        q(i) = random_double();
    }

    double gravity_norm = 9.8;
    base_acceleration.vel[2] = -gravity_norm;

    regressor_generator.setRobotState(q,q,q,base_velocity,base_acceleration);

    // Estimate sensor measurements from the model
    iDynTree::Wrench simulate_measurement = simulateFTSensorFromKinematicState(icub_undirected_tree,
        q,q,q,base_velocity,base_acceleration,"l_arm_ft_sensor",sensors_tree);


    //Create a regressor and check the returned sensor value
    Eigen::MatrixXd regressor(regressor_generator.getNrOfOutputs(),regressor_generator.getNrOfParameters());
    Eigen::VectorXd kt(regressor_generator.getNrOfOutputs());
    regressor_generator.computeRegressor(regressor,kt);

    //std::cout << "regressors : " << regressor << std::endl;

    Eigen::VectorXd parameters(regressor_generator.getNrOfParameters());
    parameters.setZero();

    regressor_generator.getModelParameters(parameters);

    assert(parameters.rows() == regressor_generator.getNrOfParameters());

    Eigen::Matrix<double,6,1> sens = regressor*parameters;

    ////////////////////////////////////////////////////////////
    ///// Test also the new iDynTree regressor infrastructure
    ////////////////////////////////////////////////////////////
    iDynTree::Regressors::DynamicsRegressorGenerator new_generator;

    // load robot and sensor model
    ok = ok && new_generator.loadRobotAndSensorsModelFromFile(icub_urdf_filename);

    assert(ok);

    // load regressor structure (this should be actually loaded from file)
    std::string regressor_structure
        = "<regressor> "
          "  <subtreeBaseDynamics> "
          "    <FTSensorLink>l_upper_arm</FTSensorLink> "
          "  </subtreeBaseDynamics> "
          "</regressor>";

    ok = ok && new_generator.loadRegressorStructureFromString(regressor_structure);

    assert(ok);

    iDynTree::VectorDynSize q_idyntree(q.rows());

    ok = ok && iDynTree::ToiDynTree(q,q_idyntree);

    assert(ok);

    iDynTree::Twist gravity = iDynTree::Twist::Zero();
    gravity.getLinearVec3()(2) = gravity_norm;

    ok = ok && new_generator.setRobotState(q_idyntree,q_idyntree,q_idyntree,gravity);

    assert(ok);

    iDynTree::MatrixDynSize regr_idyntree(new_generator.getNrOfOutputs(),new_generator.getNrOfParameters());
    iDynTree::VectorDynSize kt_idyntree(new_generator.getNrOfOutputs());
    iDynTree::VectorDynSize param_idyntree(new_generator.getNrOfParameters());

    ok = ok && new_generator.getModelParameters(param_idyntree);

    int sensorIndex = new_generator.getSensorsModel().getSensorIndex(iDynTree::SIX_AXIS_FORCE_TORQUE,"l_arm_ft_sensor");
    ok = ok && new_generator.getSensorsMeasurements().setMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,sensorIndex,simulate_measurement);

    ok = ok && new_generator.computeRegressor(regr_idyntree,kt_idyntree);

    Eigen::Matrix<double,6,1> sens_idyntree = Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(regr_idyntree.data(),regr_idyntree.rows(),regr_idyntree.cols())*
                                      Eigen::Map<Eigen::VectorXd>(param_idyntree.data(),param_idyntree.size());

    Eigen::Matrix<double,6,1> kt_eigen = Eigen::Map< Eigen::Matrix<double,6,1>  >(kt_idyntree.data(),kt_idyntree.size());

    std::cout << "Parameters norm old " << parameters.norm() << std::endl;
    std::cout << "Parameters norm new " << Eigen::Map<Eigen::VectorXd>(param_idyntree.data(),param_idyntree.size()).norm() << std::endl;
    std::cout << "Sensor measurement from regressor*model_parameters: " << sens << std::endl;
    std::cout << "Sensor measurement from regressor*model_parameters (new): " << sens_idyntree << std::endl;
    std::cout << "Sensor measurement from RNEA:                       " << KDL::CoDyCo::toEigen( iDynTree::ToKDL(simulate_measurement)) << std::endl;
    std::cout << "Sensor measurement from known terms (new) " << kt_eigen << std::endl;

    double tol = 1e-5;
    if( (KDL::CoDyCo::toEigen(iDynTree::ToKDL(simulate_measurement))+sens).norm() > tol )
    {
        std::cerr << "[ERR] iCubLeftArmRegressor error" << std::endl;
        return EXIT_FAILURE;
    }

    if( (KDL::CoDyCo::toEigen(iDynTree::ToKDL(simulate_measurement))+sens_idyntree).norm() > tol )
    {
        std::cerr << "[ERR] iCubLeftArmRegressor error: failure in new iDynTree regressor generator" << std::endl;
        return EXIT_FAILURE;
    }

    if( (KDL::CoDyCo::toEigen(iDynTree::ToKDL(simulate_measurement))+kt_eigen).norm() > tol )
    {
        std::cerr << "[ERR] iCubLeftArmRegressor error: failure in new iDynTree regressor generator" << std::endl;
        return EXIT_FAILURE;
    }


    return EXIT_SUCCESS;
}



