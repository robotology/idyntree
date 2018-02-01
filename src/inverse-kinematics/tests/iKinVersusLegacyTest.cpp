/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/InverseKinematics.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <string>
#include <iDynTree/Core/Transform.h>
#include <iCub/iKin/iKinIpOpt.h>
#include <iCub/iKin/iKinFwd.h>


using namespace yarp::os;
using namespace iCub::iKin;
using namespace yarp::sig;
using namespace yarp::math;

#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>

int main(int argc, char **argv) {

    ResourceFinder finder = ResourceFinder::getResourceFinderSingleton();
    finder.configure(argc, argv);
    finder.setVerbose(false);

    //Configure iKin Solver
    iCubLeg leg("left");
    iKinChain *chain = leg.asChain();

    for (int i = 0; i < 6; i++) {
        chain->setAng(i, 0);
    }

    std::cerr << "Initial " << chain->EndEffPose().toString() << "\n";


    std::cerr << "------------ iKin ------------ \n";
    std::cerr << "Initial joint configurations\n" << chain->getAng().toString() << "\n";
    Vector xhat = chain->EndEffPose().subVector(0, 2);
    std::cerr << "Initial pose\n" << xhat.toString() << "\n";

    iKinIpOptMin slv(*chain, IKINCTRL_POSE_FULL, 1e-3, 1e-6, 100);
    slv.setUserScaling(true, 100.0, 100.0, 100.0);

    Vector xf(7,0.0);
    xf[0]= -0.4e-5;
    xf[1]= -0.068097;
    xf[2]= -0.5935;
    xf[3]= 0.4e-5;
    xf[4]= -1;
    xf[5]= 0.3e-5;
    xf[6]= M_PI;


    Vector solution = slv.solve(chain->getAng(), xf);
    xhat = chain->EndEffPose().subVector(0, 2);
    xf = xf.subVector(0, 2);
    double dist = norm(xf - xhat);

    printf("target position = (%s) [m]\n",xf.toString(5,5).c_str());
    printf("solved position = (%s) [m]\n",xhat.toString(5,5).c_str());
    printf("distance to target = %g [m] ...\n",dist);


    std::cerr << "Final joints\n" << solution.toString() << "\n";
    std::cerr << "------------ END iKin ------------ \n\n\n";


    return 0;


//    //Configure iKin Solver
//    iCubArm arm("right");
//    iKinChain *chain = arm.asChain();
//
//    std::cerr << "------------ iKin ------------ \n";
//    std::cerr << "Initial joint configurations\n" << chain->getAng().toString() << "\n";
//    Vector xhat = chain->EndEffPose().subVector(0, 2);
//    std::cerr << "Initial pose\n" << xhat.toString() << "\n";
//
//    iKinIpOptMin slv(*chain, IKINCTRL_POSE_XYZ, 1e-3, 1e-6, 100);
//    slv.setUserScaling(true, 100.0, 100.0, 100.0);
//
//    Vector xf(7,0.0);
//    xf[0]=-0.3;
//    xf[1]=+0.1;
//    xf[2]=+0.1;
//
//    Vector solution = slv.solve(chain->getAng(), xf);
//    xhat = chain->EndEffPose().subVector(0, 2);
//    xf = xf.subVector(0, 2);
//    double dist = norm(xf - xhat);
//
//    printf("target position = (%s) [m]\n",xf.toString(5,5).c_str());
//    printf("solved position = (%s) [m]\n",xhat.toString(5,5).c_str());
//    printf("distance to target = %g [m] ...\n",dist);
//
//
//    std::cerr << "Final joints\n" << solution.toString() << "\n";
//    std::cerr << "------------ END iKin ------------ \n\n\n";



    //Temp: to be removed
    Network::setEnvironment("YARP_DATA_DIRS", "/Users/makaveli/Projects/src/local/share/yarp:/Users/makaveli/Projects/src/local/share/iCub:/Users/makaveli/Projects/src/local/../codyco-superbuild/build/install/share/codyco");

    if (finder.check("robotName") && finder.find("robotName").isString()) {
        std::string robotName = finder.find("robotName").asString();
        Network::setEnvironment("YARP_ROBOT_NAME", robotName);
    }

    if (!finder.check("model") || !finder.find("model").isString()) {
        yError("Option 'model' is required");
        return -1;
    }
    std::string urdfModelFile = finder.findFile("model");

    //allocate inverse kinematics object
    kinematics::InverseKinematics robotIKin;
    if (!robotIKin.loadModelFromURDFFileWithName(urdfModelFile)) {
        yError("Cannot load model from %s", urdfModelFile.c_str());
    }
    
    //Limits are taken from URDF, but they can be overwritten
    //robotIKin.setJointLimit();

    std::cerr << "--------- FloatingKin -----------\n\n";
    
    std::cerr << "Initial configuration\n";
    iDynTree::Transform pose;
    robotIKin.getPoseForFrame("r_hand", pose);
    std::cerr << "Pose: " << pose.getPosition().toString() << "\n";

    robotIKin.getPoseForFrame("root_link", pose);
    std::cerr << "Root: " << pose.toString() << "\n\n";

    //I want to set the right arm configuration to be (right arm)
    //-1.666789	 0.000000	-0.645772	 0.095993	-0.872665	-1.134464	-0.436332
    robotIKin.setJointConfiguration("r_shoulder_pitch", -1.666789);
    robotIKin.setJointConfiguration("r_shoulder_roll", 0.000000);
    robotIKin.setJointConfiguration("r_shoulder_yaw", -0.645772);
    robotIKin.setJointConfiguration("r_elbow", 0.095993);
    robotIKin.setJointConfiguration("r_wrist_prosup", -0.872665);
    robotIKin.setJointConfiguration("r_wrist_pitch", -1.134464);
    robotIKin.setJointConfiguration("r_wrist_yaw", -0.436332);

    robotIKin.getPoseForFrame("r_hand", pose);
    std::cerr << "Pose: " << pose.getPosition().toString() << "\n";
    std::cerr << "Rot: " << pose.getRotation().toString() << "\n";

    robotIKin.getPoseForFrame("root_link", pose);
    std::cerr << "Root: " << pose.toString() << "\n";

    
    //Set initial configuration for the robot
    //This must match the #Dofs of the URDF
    //robotIKin.setRobotConfiguration(qj);
    
    //Optionally: set joint map.
    //This is used to specify the optimization variables.
    //Joints not present in this list are not optimized
    std::vector<std::string> map;
    map.push_back("r_shoulder_pitch");
    map.push_back("r_shoulder_roll");
    map.push_back("r_shoulder_yaw");
    map.push_back("r_elbow");
    map.push_back("r_wrist_prosup");
    map.push_back("r_wrist_pitch");
    map.push_back("r_wrist_yaw");
//    robotIKin.setOptimizationVariablesToJointsMapping(map);

    //add constraints on links
    //two versions: one with name of the frame and homogeneous transformation
    //second one with only the name of the frame. In this case the frame is kept constant.
    //robotIKin.addFrameConstraint("root_link", w_X_r);
//    robotIKin.addFrameConstraint("lower_link");
    robotIKin.addFrameConstraint("root_link");
//    robotIKin.addFrameConstraint("base");

    //Specify the target (or more than one?) frames
    robotIKin.getPoseForFrame("r_hand", pose);

    iDynTree::Position w_p_lh(0.3, 0.1, 0.1);
    iDynTree::Rotation w_R_lh = iDynTree::Rotation::Identity();
    iDynTree::Transform w_X_lh(w_R_lh, w_p_lh);
    robotIKin.addTarget("r_hand", pose);

    //robotIKin.setInitialCondition(x0);

    //test left end effector

//    robotIKin.setRotationParametrization(kinematics::InverseKinematicsRotationParametrizationRollPitchYaw);

    robotIKin.solve();


    return 0;
    
}
