#include <iCub/iDynTree/DynTree.h>
#include <iDynTree/HighLevel/DynamicsComputations.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/Transform.h>
#include <kdl/frames_io.hpp>
#include <yarp/sig/Vector.h>

#include <cstdlib>

double random_double()
{
    return  3.1415/2;
}


bool checkFrameConsistency(KDL::Frame & frame_kdl,
                           iDynTree::Transform & trans_idyntree,
                           double tol = 1e-9
                          )
{
    iDynTree::Rotation rot_idyn = trans_idyntree.getRotation();
    KDL::Rotation      rot_kdl = frame_kdl.M;
    for(int row = 0; row < 3; row++)
    {
        for(int col = 0; col < 3; col++)
        {
            if( fabs(rot_idyn(row,col)-rot_kdl(row,col)) > tol )
            {
                return false;
            }
        }
    }


    iDynTree::Position pos_idyn = trans_idyntree.getPosition();
    KDL::Vector      pos_kdl = frame_kdl.p;
    for(int row = 0; row < 3; row++)
    {
        if( fabs(pos_idyn(row)-pos_kdl(row)) > tol )
        {
            return false;
        }
    }
    return true;
}

int main()
{
    std::string urdf_to_test = "twoLinks.urdf";

    srand(time(NULL));

    //Creating an DynTree
    iCub::iDynTree::DynTree urdf_dyntree;
    bool ok = urdf_dyntree.loadURDFModel(urdf_to_test);

    //Creating a DynamicsComputations class
    iDynTree::HighLevel::DynamicsComputations urdf_dyncomp;
    ok = ok && urdf_dyncomp.loadRobotModelFromFile(urdf_to_test);

    if( !ok )
    {
        std::cerr << "Impossible to load URDF " << urdf_to_test << std::endl;
        return EXIT_FAILURE;
    }

    // Create random joint configuration
    unsigned int dofs = urdf_dyncomp.getNrOfDegreesOfFreedom();
    iDynTree::VectorDynSize q_idyntree(dofs),zero_idyntree(dofs);
    iDynTree::SpatialAcc gravity;
    yarp::sig::Vector q_yarp(dofs);

    for(int i=0; i < dofs; i++ )
    {
        q_yarp(i) = q_idyntree(i) = random_double();
    }

    urdf_dyntree.setAng(q_yarp);
    urdf_dyncomp.setRobotState(q_idyntree,zero_idyntree,zero_idyntree,gravity);

    //Check consistenct of all the transformations
    unsigned int nrOfFrames = urdf_dyncomp.getNrOfFrames();
    for(unsigned refFrame = 0; refFrame < nrOfFrames; refFrame++ )
    {
        for(unsigned frame = 0; frame < nrOfFrames; frame++ )
        {
            KDL::Frame H_kdl = urdf_dyntree.getPositionKDL(refFrame,(int)frame);
            iDynTree::Transform H_transform = urdf_dyncomp.getRelativeTransform(refFrame,frame);

            if( !checkFrameConsistency(H_kdl,H_transform) )
            {
                std::cerr << "Error : " << refFrame << "_T_" << frame << " is inconsistent " << std::endl;
                std::cerr << "DynamicsComputations : " << H_transform.toString() << std::endl;
                std::cerr << "DynTree                " << H_kdl << std::endl;
                return EXIT_FAILURE;
            }
        }
    }

    // The same, but with names
    nrOfFrames = urdf_dyncomp.getNrOfFrames();
    for(unsigned refFrame = 0; refFrame < nrOfFrames; refFrame++ )
    {
        for(unsigned frame = 0; frame < nrOfFrames; frame++ )
        {

            std::string refFrameName, frameName;
            urdf_dyntree.getFrameName(refFrame,refFrameName);
            urdf_dyntree.getFrameName(frame,frameName);
            KDL::Frame H_kdl = urdf_dyntree.getPositionKDL(refFrame,(int)frame);
            iDynTree::Transform H_transform = urdf_dyncomp.getRelativeTransform(refFrameName,frameName);

            if( !checkFrameConsistency(H_kdl,H_transform) )
            {
                std::cerr << "Error : " << refFrameName << "_T_" << frameName << " is inconsistent " << std::endl;
                std::cerr << "DynamicsComputations : " << H_transform.toString() << std::endl;
                std::cerr << "DynTree                " << H_kdl << std::endl;
                return EXIT_FAILURE;
            }
            if( refFrame != frame )
            {
                std::cerr << "Check: " << refFrameName << "_T_" << frameName << " is inconsistent " << std::endl;
                std::cerr << "DynamicsComputations : " << H_transform.toString() << std::endl;
                std::cerr << "DynTree                " << H_kdl << std::endl;
                std::cerr << "Rotation " << std::endl;
                std::cerr << "DynamicsComputations rot: " << H_transform.getRotation().toString() << std::endl;
                std::cerr << "DynTree rot               " << H_kdl.M << std::endl;
                for(int row = 0; row <3; row++ )
                {
                    for(int col = 0; col < 3; col++ )
                    {

                    }
                }
            }
        }
    }


    return EXIT_SUCCESS;
}
