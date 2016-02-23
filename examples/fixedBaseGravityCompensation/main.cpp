/**
* @ingroup idyntree_tutorials
*
*
* A tutorial on how to the gravity compensation terms for a fixed
*  base manipulator.
*
* \author Silvio Traversaro
*
* CopyPolicy: Released under the terms of GPL 2.0 or later
*/

// YARP headers
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>

// iDynTree headers
#include <iCub/iDynTree/DynTree.h>
#include <urdf_exception/exception.h>

#include <string>
#include <iostream>

/*****************************************************************/
int main(int argc, char *argv[])
{
    // In this example we use yarp to parse the configuration
    // parameters
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("name","fixedBaseGravityCompensation");
    rf.setDefault("config","config.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo() << "Options:\n\n";
        yInfo() << "\t--urdf    file: name of the URDF file containing"
                   " the model of the robot.  \n";

        return 0;
    }

    std::string urdf_filename = rf.findFile("urdf");


    yInfo() << "Tryng to open " << urdf_filename << " as robot model";
    iCub::iDynTree::DynTree robot_model;
    bool ok = robot_model.loadURDFModel(urdf_filename);

    if( !ok )
    {
        std::cerr << "Loading urdf file " << urdf_filename << " failed, exiting" << std::endl;
        return EXIT_FAILURE;
    }

    //
    // Now we will set the variable necessary to compute the gravity compensation terms
    //

    // The gravity acceleration vector should be expressed in the base link frame,
    // in this example we assume that this frame has the Z axis pointing up
    // All the inputs are expressed in SI unit of measure, so the gravity acceleration
    // is expressed in m/s^2
    yarp::sig::Vector grav(3);
    grav(0) = grav(1) = 0.0;
    grav(2) = -9.8;

    robot_model.setGravity(grav);

    // We now need to set the joint position for the desired gravity compensation
    // For the sake of example, we will put all the joint positions to 10.0 degrees;
    yarp::sig::Vector q(robot_model.getNrOfDOFs());

    double deg2rad = 3.14/180.0;

    for(int i =0; i < q.size(); i++ )
    {

        q(i) = deg2rad*10.0;
    }

    // Now we set the joint values
    robot_model.setAng(q);

    // Now we can call the RNEA algorithm to compute the gravity torques
    robot_model.kinematicRNEA();
    robot_model.dynamicRNEA();

    // The obtained torques are expressed in Nm
    yarp::sig::Vector gravity_torques = robot_model.getTorques();

    // Output some result
    std::cout << "Input gravity : " << grav.toString() << std::endl;
    std::cout << "Input joint positions : " << robot_model.getAng().toString() << std::endl;
    std::cout << "Output joint torques  : " << gravity_torques.toString() << std::endl;


    return EXIT_SUCCESS;
}

