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

#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Regressors/DynamicsRegressorGenerator.h>

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

    std::string urdf_filename = rf.findFile("../bigman_left_arm.urdf");
    yInfo() << "Trying to open " << urdf_filename << " as robot model";

    iDynTree::Regressors::DynamicsRegressorGenerator generator;
    generator.loadRobotAndSensorsModelFromFile(urdf_filename);

    const char* regrXml =
        "<regressor>"
        "    <jointTorqueDynamics>"
        "      <joints>"
        "        <joint>LShSag</joint>"
        "        <joint>LShLat</joint>"
        "        <joint>LShYaw</joint>"
        "        <joint>LElbj</joint>"
        "        <joint>LForearmPlate</joint>"
        "        <joint>LWrj1</joint>"
        "        <joint>LWrj2</joint>"
        "      </joints>"
        "    </jointTorqueDynamics>"
        "</regressor>";

    generator.loadRegressorStructureFromString(regrXml);

    std::cout << generator.getDescriptionOfParameters() << std::endl;

    uint N_DOFS = generator.getNrOfDegreesOfFreedom();
    std::cout << "Nr of DOFs:" << N_DOFS << std::endl;
    uint N_OUT = generator.getNrOfOutputs();
    std::cout << "Nr of outputs:" << N_OUT << std::endl;
    uint N_LINKS = generator.getNrOfLinks();
    std::cout << "Nr of links:" << N_LINKS << std::endl;
    std::cout << "Nr of fake links:" << generator.getNrOfFakeLinks() << std::endl;
    uint N_PARAMS = generator.getNrOfParameters();
    std::cout << "Nr of params:" << N_PARAMS << std::endl;
    iDynTree::VectorDynSize cadParams(N_PARAMS);
    generator.getModelParameters(cadParams);

    iDynTree::Twist gravity_twist;
    gravity_twist.zero();
    gravity_twist.setVal(2, -9.81);

    iDynTree::VectorDynSize qq (N_DOFS);
    iDynTree::VectorDynSize dq (N_DOFS);
    iDynTree::VectorDynSize ddq (N_DOFS);

    /*for (uint dof=0; dof<N_DOFS; dof++) {
        qq.setVal(dof, 1.0);
        dq.setVal(dof, 0.01);
        ddq.setVal(dof, 0.001);
    }*/

    generator.setRobotState(qq,dq,ddq, gravity_twist);
    generator.setTorqueSensorMeasurement(0, 0.05);

    iDynTree::MatrixDynSize regressor(N_OUT, N_PARAMS);  //Y
    iDynTree::VectorDynSize knownTerms(N_OUT);  //corresponds to measured torques in this case
    if (!generator.computeRegressor(regressor, knownTerms)) {
        std::cout << "Error while computing regressor" << std::endl;
    }

    std::cout << regressor.toString() << std::endl;


    return EXIT_SUCCESS;
}

