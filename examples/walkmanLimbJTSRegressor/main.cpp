/**
* @ingroup idyntree_tutorials
*
*
* A tutorial on how to obtain the parametric model for
*  fixed base Walkman limb joint torque sensor measurement.
*
* \author Silvio Traversaro
*
* CopyPolicy: Released under the terms of GPL 2.0 or later
*/

// YARP headers
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>

// iDynTree headers
#include <iDynTree/Sensors/Sensors.hpp>>
#include <iDynTree/Regressors/DynamicsRegressorGenerator.h>
#include <iDynTree/ModelIO/impl/urdf_import.hpp>

#include <string>
#include <iostream>

/*****************************************************************/
int main(int argc, char *argv[])
{
    // In this example we will use YARP only for parsing command line
    // parameters 
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("name","walkmanLimbJTSRegressor");
    rf.setDefault("config","config.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo() << "Options:\n\n";
        yInfo() << "\t--urdf    file: name of the URDF file containing"
                   " the model of the Walkman.  \n";

        return 0;
    }

    std::string urdf_filename = rf.findFile("urdf");

    yInfo() << "Tryng to open " << urdf_filename << " as Walkman model";

    /**
      * The dynamics regressor generator is a class for calculating arbitrary regressor
      * related to robot dynamics, for identification of dynamics parameters, such
      * as inertial parameters (masses, centers of mass, inertia tensor elements).
      */

    //For building the regressor generator, we need several inputs:
    // * the model of the robot, given by the KDL::CoDyCo::UndirectedTree object
    // * a model of the sensors of the robot, given by the KDL::CoDyCo::SensorsTree object
    //   (at this moment the SensorsTree supports only six-axis FT sensors, so it is not needed
    //    for joint torque sensors regressors)
    // * a kinematic base (i.e.  link for which the velocity/acceleration and the gravity are known:
    //                     in this example we will leave the default one, i.e. the root of the robot)
    // * the option for considering or not FT sensors offset in the parameter estimation
    //                     (in this example we don't consider FT sensors, so we put this to false)
    // * a list of "fake_links", for which no inertial parameters will be considered, useful for removing
    //   from the model "fake" links such as frames.


    KDL::Tree walkman_tree;
    if( !kdl_format_io::treeFromUrdfFile(urdf_filename,walkman_tree) )
    {
        std::cerr << " Could not parse urdf robot model" << std::endl;
        return EXIT_FAILURE;
    }

    KDL::CoDyCo::UndirectedTree walkman_undirected_tree(walkman_tree);


    KDL::CoDyCo::SensorsTree walkman_sensors_tree;
    std::string kinematic_base = "";
    bool consider_ft_offsets = false;
    std::vector< std::string > fake_links;
    KDL::CoDyCo::Regressors::DynamicRegressorGenerator jts_regressor_generator(walkman_undirected_tree,
                                                                               walkman_sensors_tree,
                                                                               kinematic_base,
                                                                               consider_ft_offsets,
                                                                               fake_links);

    // We add the structure now: the list of joint torque sensors that we are considering
    std::vector< std::string > jts_names;

    for(int jts = 0; jts < jts_names.size(); jts++ )
    {
        jts_regressor_generator.addTorqueRegressorRows(jts_names[jts]);
    }

    // Now we can print some information about the regressor
    std::cout << "Regressor information: " << std::endl;
    std::cout << "Outputs: " << std::endl;
    std::cout << jts_regressor_generator.getDescriptionOfOutputs() << std::endl;
    std::cout << "Parameters: " << std::endl;
    std::cout << jts_regressor_generator.getDescriptionOfParameters() << std::endl;

    // We now have to set the robot state (in the fixed base case);
    KDL::JntArray q(walkman_undirected_tree.getNrOfDOFs());
    KDL::JntArray dq(walkman_undirected_tree.getNrOfDOFs());
    KDL::JntArray ddq(walkman_undirected_tree.getNrOfDOFs());

    // For this example we will consider all joint positions, velocities and accelerations to zero
    KDL::SetToZero(q);
    KDL::SetToZero(dq);
    KDL::SetToZero(ddq);

    KDL::Vector gravity;
    gravity(2) = -9.8;

    KDL::Twist gravity_twist(gravity,KDL::Vector::Zero());

    jts_regressor_generator.setRobotState(q,dq,ddq,gravity_twist);

    // Now we can generate the actual regressor
    Eigen::MatrixXd regressor(jts_regressor_generator.getNrOfOutputs(),jts_regressor_generator.getNrOfParameters());
    Eigen::VectorXd known_terms(jts_regressor_generator.getNrOfOutputs());
    jts_regressor_generator.computeRegressor(regressor,known_terms);

    return 0;

}

