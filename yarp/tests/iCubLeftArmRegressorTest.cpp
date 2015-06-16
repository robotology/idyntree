/*
* Copyright (C) 2015 RBCS Department, Istituto Italiano di Tecnologia
* Author: Silvio Traversaro <silvio.traversaro@iit.it>
*
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/
//

#include <kdl/tree.hpp>
#include <kdl_codyco/undirectedtree.hpp>
#include <iDynTree/Sensors/Sensors.hpp>
#include <iDynTree/Sensors/SixAxisFTSensor.hpp>

#include <kdl_codyco/KDLConversions.h>

#include <iDynTree/Core/Transform.h>

#include <kdl_format_io/urdf_import.hpp>
#include <kdl_format_io/urdf_sensor_import.hpp>

#include <kdl_codyco/regressors/dynamicRegressorGenerator.hpp>
#include <kdl_codyco/regressor_loops.hpp>
#include <kdl_codyco/rnea_loops.hpp>

#include <kdl_codyco/regressor_utils.hpp>
#include <kdl_codyco/regressors/dirl_utils.hpp>

#include <iostream>
#include <string>
#include <vector>

#include <cstdlib>

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
iDynTree::SensorsList sensorsTreeFromURDF(KDL::CoDyCo::UndirectedTree & undirected_tree,
                                             std::string urdf_filename)
{
    iDynTree::SensorsList sensors_tree;

    std::vector<kdl_format_io::FTSensorData> ft_sensors;
    bool ok = kdl_format_io::ftSensorsFromUrdfFile(urdf_filename,ft_sensors);

    if( !ok )
    {
        std::cerr << "Error in loading ft sensors information from URDF file" << std::endl;
    }

    for(int ft_sens = 0; ft_sens < ft_sensors.size(); ft_sens++ )
    {
        iDynTree::SixAxisForceTorqueSensor new_sens;

        // Convert the information in the FTSensorData format in
        // a series of SixAxisForceTorqueSensor objects, using the
        // serialization provided in the undirected_tree object
        new_sens.setName(ft_sensors[ft_sens].reference_joint);

        new_sens.setParent(ft_sensors[ft_sens].reference_joint);

        KDL::CoDyCo::JunctionMap::const_iterator junct_it
            = undirected_tree.getJunction(ft_sensors[ft_sens].reference_joint);

        new_sens.setParentIndex(junct_it->getJunctionIndex());

        int parent_link = junct_it->getParentLink()->getLinkIndex();
        int child_link = junct_it->getChildLink()->getLinkIndex();

        KDL::Frame parent_link_H_child_link = junct_it->pose(0.0,false);
        KDL::Frame child_link_H_sensor = ft_sensors[ft_sens].sensor_pose;

        // For now we assume that the six axis ft sensor is attached to a
        // fixed junction. Hence the first/second link to sensor transforms
        // are fixed are given by the frame option
        if( ft_sensors[ft_sens].frame == kdl_format_io::FTSensorData::PARENT_LINK_FRAME )
        {
            new_sens.setFirstLinkSensorTransform(parent_link,iDynTree::Transform());
            new_sens.setSecondLinkSensorTransform(child_link,iDynTree::ToiDynTree(parent_link_H_child_link.Inverse()));
        }
        else if( ft_sensors[ft_sens].frame == kdl_format_io::FTSensorData::CHILD_LINK_FRAME )
        {
            new_sens.setFirstLinkSensorTransform(parent_link,iDynTree::ToiDynTree(parent_link_H_child_link));
            new_sens.setSecondLinkSensorTransform(child_link,iDynTree::Transform());
        }
        else
        {
            assert( ft_sensors[ft_sens].frame == kdl_format_io::FTSensorData::SENSOR_FRAME );
            new_sens.setFirstLinkSensorTransform(parent_link,iDynTree::ToiDynTree(parent_link_H_child_link*child_link_H_sensor));
            new_sens.setSecondLinkSensorTransform(child_link,iDynTree::ToiDynTree(child_link_H_sensor));
        }

        if( ft_sensors[ft_sens].measure_direction == kdl_format_io::FTSensorData::CHILD_TO_PARENT )
        {
            new_sens.setAppliedWrenchLink(parent_link);
        }
        else
        {
            assert( ft_sensors[ft_sens].measure_direction == kdl_format_io::FTSensorData::CHILD_TO_PARENT );
            new_sens.setAppliedWrenchLink(child_link);
        }

        sensors_tree.addSensor(new_sens);
    }

    return sensors_tree;
}

int main()
{
    std::string icub_urdf_filename = "icub_model.urdf";

    // Create the iCub model
    KDL::Tree icub_tree;
    bool ok = kdl_format_io::treeFromUrdfFile(icub_urdf_filename,icub_tree);

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

    // Load a sensors tree (for ft sensors) from the information extracted from urdf file
    //  and using the serialization provided in the undirected tree
    iDynTree::SensorsList sensors_tree = sensorsTreeFromURDF(icub_undirected_tree,icub_urdf_filename);

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
    for(int i=0; i < q.rows(); i++ )
    {
        q(i) = random_double();
    }

    base_acceleration.vel[2] = -9.81;

    regressor_generator.setRobotState(q,q,q,base_velocity,base_acceleration);

    // Estimate sensor measurements from the model
    iDynTree::Wrench simulate_measurement = simulateFTSensorFromKinematicState(icub_undirected_tree,
        q,q,q,base_velocity,base_acceleration,"l_arm_ft_sensor",sensors_tree);

        
    //Create a regressor and check the returned sensor value
    Eigen::MatrixXd regressor(regressor_generator.getNrOfOutputs(),regressor_generator.getNrOfParameters());
    Eigen::VectorXd kt(regressor_generator.getNrOfOutputs());
    regressor_generator.computeRegressor(regressor,kt);

    std::cout << "regressors : " << regressor << std::endl;

    Eigen::VectorXd parameters(regressor_generator.getNrOfParameters());
    parameters.setZero();

    std::vector<std::string> fake_links;
    KDL::CoDyCo::inertialParametersVectorLoopFakeLinks(icub_undirected_tree,parameters,fake_links);

    assert(parameters.rows() == regressor_generator.getNrOfParameters());

    Eigen::Matrix<double,6,1> sens = regressor*parameters;

    std::cout << "Sensor measurement from regressor*model_parameters: " << sens << std::endl;
    std::cout << "Sensor measurement from RNEA:                       " << KDL::CoDyCo::toEigen( iDynTree::ToKDL(simulate_measurement)) << std::endl;

    double tol = 1e-5;
    if( (KDL::CoDyCo::toEigen(iDynTree::ToKDL(simulate_measurement))+sens).norm() > tol )
    {
        std::cerr << "[ERR] iCubLeftArmRegressor error" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}



