/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Silvio Traversaro <silvio.traversaro@iit.it>
 */

#include <cstdlib>
#include <yarp/os/all.h>

#include "robotstatepublisher.h"

using namespace std;


/****************************************************************/
int main(int argc, char *argv[])
{
    // initialize yarp network:
    // the very first thing to do
    yarp::os::Network yarp;

    // load command-line options
    yarp::os::ResourceFinder rf;
    rf.configure(argc,argv);

    // print available command-line options
    // upon specifying --help; refer to xml
    // descriptor for the relative documentation
    if (rf.check("help"))
    {
        cout<<"Options"<<endl;
        cout<<"\t--name-prefix            <name-prefix> : prefix of the yarprobotstatepublisher ports (default no prefix)"<<endl;
        cout<<"\t--tf-prefix              <tf-prefix>   : prefix of the published TFs (default no prefix)"<<endl;
        cout<<"\t--model                  <file-name>   : file name of the model to load at startup"<<endl;
        cout<<"\t--reduced-model                        : use the option to stream only the link TFs\n"
              "                                           \t By default TFs of all the frames in the model are streamed"<<endl;
        cout<<"\t--base-frame             <frame-name>  : specify the base frame of the published tf tree"<<endl;
        cout<<"\t--jointstates-topic      <topic-name>  : source ROS topic that streams the joint state (default /joint_states)\n"
              "                                           \t The position values of the model joints are initilized to Zero\n"
              "                                           \t In runtime, the joint values from the ROS topic are used to set\n"
              "                                           \t the position values of some of the model joints."<<endl;
        cout<<"\t--tree-type              <tree-type>   : the type of tree tou want to represent the transformations in.\n"
              "                                           \t the values can be DEEP or SHALLOW. If omitted, the default\n"
              "                                           \t value SHALLOW will be used"<<endl;
        return EXIT_SUCCESS;
    }

    if (!yarp.checkNetwork(10.0))
    {
        yError()<<"YARP network is not available";
        return EXIT_FAILURE;
    }

    YARPRobotStatePublisherModule statepublisher;
    return statepublisher.runModule(rf);
}
