/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "testModels.h"

#include <iDynTree/HighLevel/DynamicsComputations.h>
#include <iDynTree/Model/FreeFloatingMatrices.h>

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>

#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/ClassicalAcc.h>
#include <iDynTree/Core/SpatialAcc.h>

#include <iDynTree/Core/EigenHelpers.h>

#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <iCub/iDynTree/DynTree.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>


using namespace iDynTree;

double random_double()
{
    return ((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

double actual_random_double()
{
    return ((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

void yarp2idyntree(const yarp::sig::Matrix & yarpMatrix,
                   iDynTree::MatrixDynSize & idyntreeMatrix)
{
    idyntreeMatrix.resize(yarpMatrix.rows(),yarpMatrix.cols());
    for(size_t row=0; row < idyntreeMatrix.rows(); row++)
    {
        for(size_t col=0; col < idyntreeMatrix.cols(); col++)
        {
            idyntreeMatrix(row,col) = yarpMatrix(row,col);
        }
    }
}

void yarp2idyntree(const yarp::sig::Vector & yarpVector,
                   iDynTree::VectorDynSize & idyntreeVector)
{
    idyntreeVector.resize(yarpVector.size());
    for(size_t row=0; row < idyntreeVector.size(); row++)
    {
        idyntreeVector(row) = yarpVector(row);
    }

    return;
}

template<typename iDynTreeMatrixType>
void idyntree2yarp(const iDynTreeMatrixType & idyntreeMatrix,
                   yarp::sig::Matrix & yarpMatrix)
{
    yarpMatrix.resize(idyntreeMatrix.rows(),idyntreeMatrix.cols());
    for(size_t row=0; row < idyntreeMatrix.rows(); row++)
    {
        for(size_t col=0; col < idyntreeMatrix.cols(); col++)
        {
            yarpMatrix(row,col) = idyntreeMatrix(row,col);
        }
    }
}

template<typename iDynTreeMatrixType>
yarp::sig::Matrix idyntreeMat2yarp(const iDynTreeMatrixType & idyntreeMatrix)
{
    yarp::sig::Matrix yarpMatrix;
    idyntree2yarp(idyntreeMatrix,yarpMatrix);
    return yarpMatrix;
}

template<typename iDynTreeVectorType>
void idyntreeMat2yarp(const iDynTreeVectorType & idyntreeVector,
                   yarp::sig::Vector & yarpVector)
{
    yarpVector.resize(idyntreeVector.size());
    for(size_t row=0; row < idyntreeVector.size(); row++)
    {
        yarpVector(row) = idyntreeVector(row);
    }
}

template<typename iDynTreeVectorType>
yarp::sig::Vector idyntree2yarp(const iDynTreeVectorType & idyntreeVector)
{
    yarp::sig::Vector yarpVector;
    yarpVector.resize(idyntreeVector.size());
    for(size_t row=0; row < idyntreeVector.size(); row++)
    {
        yarpVector(row) = idyntreeVector(row);
    }
    return yarpVector;
}

iDynTree::Transform yarpTransform2idyntree(yarp::sig::Matrix transformYarp)
{
    ASSERT_EQUAL_DOUBLE(transformYarp.rows(),4);
    ASSERT_EQUAL_DOUBLE(transformYarp.cols(),4);

    Rotation R(transformYarp(0,0),transformYarp(0,1),transformYarp(0,2),
               transformYarp(1,0),transformYarp(1,1),transformYarp(1,2),
               transformYarp(2,0),transformYarp(2,1),transformYarp(2,2));

    Position p(transformYarp(0,3),transformYarp(1,3),transformYarp(2,3));

    return iDynTree::Transform(R,p);
}

void setRandomState(iDynTree::HighLevel::DynamicsComputations & dynComp,
                    iCub::iDynTree::DynTree & dynTree,
                    iDynTree::KinDynComputations& kinDynComp)
{
   std::cerr << " setRandomState" << std::endl;
    
    size_t dofs = dynComp.getNrOfDegreesOfFreedom();
    Transform    worldTbase;
    Twist        baseVel;
    ClassicalAcc baseAcc;
    SpatialAcc gravity;
    Vector6    properAcc;

    iDynTree::VectorDynSize qj(dofs), dqj(dofs), ddqj(dofs);

    worldTbase = iDynTree::Transform(Rotation::RPY(random_double(),random_double(),random_double()),
                                     Position(random_double(),random_double(),random_double()));
    for(int i=0; i < 3; i++)
    {
        gravity(i) = random_double();
    }

    gravity(2) = 10.0;

    for(int i=0; i < 6; i++)
    {
        baseVel(i) = random_double();
        baseAcc(i) = random_double();
        properAcc(i) = baseAcc(i) - gravity(i);
    }

    for(size_t dof=0; dof < dofs; dof++)

    {
        qj(dof) = random_double();
        dqj(dof) = random_double();
        ddqj(dof) = random_double();
    }

    bool ok = dynComp.setRobotState(qj,dqj,ddqj,worldTbase,baseVel,baseAcc,gravity);
    
    ASSERT_IS_TRUE(ok);

    dynTree.setAng(idyntree2yarp(qj));
    dynTree.setDAng(idyntree2yarp(dqj));
    dynTree.setD2Ang(idyntree2yarp(ddqj));
    dynTree.setWorldBasePose(idyntreeMat2yarp(worldTbase.asHomogeneousTransform()));
    dynTree.setKinematicBaseVelAcc(idyntree2yarp(baseVel),idyntree2yarp(properAcc));

    std::cerr << " Setting state in kinDyn" << std::endl;
    ASSERT_EQUAL_DOUBLE(qj.size(),kinDynComp.getRobotModel().getNrOfPosCoords());
    ASSERT_EQUAL_DOUBLE(dqj.size(),kinDynComp.getRobotModel().getNrOfDOFs());

    Vector3 grav3d = gravity.getLinearVec3();
    ok = kinDynComp.setRobotState(worldTbase,qj,baseVel,dqj,grav3d);
    ASSERT_IS_TRUE(ok);
    
    std::cerr << "state setted" << std::endl;
}

void testTransformsConsistency(iDynTree::HighLevel::DynamicsComputations & dynComp,
                               iCub::iDynTree::DynTree & dynTree,
                               iDynTree::KinDynComputations & kinDynComp)
{
    for(size_t frame=0; frame < dynComp.getNrOfFrames(); frame++ )
    {
        std::string frameNameDynComp;
        std::string frameNameDynTree;

        dynTree.getFrameName(frame,frameNameDynTree);
        frameNameDynComp = dynComp.getFrameName(frame);

        ASSERT_EQUAL_STRING(frameNameDynComp,frameNameDynTree);

        Transform dynTreeTransform = yarpTransform2idyntree(dynTree.getPosition(frame));
        Transform dynCompTransform = dynComp.getWorldTransform(frame);

        ASSERT_EQUAL_TRANSFORM(dynCompTransform,dynTreeTransform);
        
        Transform kinDynCompTransform = kinDynComp.getWorldTransform(frameNameDynComp);
        
        ASSERT_EQUAL_TRANSFORM(dynCompTransform,kinDynCompTransform);
    }
}

void testJacobianConsistency(iDynTree::HighLevel::DynamicsComputations & dynComp,
                             iCub::iDynTree::DynTree & dynTree,
                             iDynTree::KinDynComputations & kinDynComp)
{
    size_t dofs = dynComp.getNrOfDegreesOfFreedom();
    MatrixDynSize dynTreeJacobian(6,6+dofs),dynCompJacobian(6,6+dofs);
    yarp::sig::Matrix        dynTreeJacobianYarp(6,6+dofs);
    FrameFreeFloatingJacobian kinDynCompJacobian(kinDynComp.getRobotModel());

    for(size_t frame=0; frame < dynComp.getNrOfFrames(); frame++ )
    {
        dynTree.getJacobian(frame,dynTreeJacobianYarp);
        yarp2idyntree(dynTreeJacobianYarp,dynTreeJacobian);

        dynComp.getFrameJacobian(frame,dynCompJacobian);
        
        std::string frameName = dynComp.getFrameName(frame);

        ASSERT_EQUAL_MATRIX(dynTreeJacobian,dynCompJacobian);
        
        std::cerr << "Testing frame " << frameName << std::endl;
        
        bool ok = kinDynComp.getFrameFreeFloatingJacobian(frameName,kinDynCompJacobian);
        
        std::cerr << "DynamicsComputations: " << std::endl;
        std::cerr << dynCompJacobian.toString() << std::endl;
        std::cerr << "KinamicsDynamicsComputations : " << std::endl;
        std::cerr << kinDynCompJacobian.toString() << std::endl;
        
        ASSERT_IS_TRUE(ok);
        ASSERT_EQUAL_MATRIX(kinDynCompJacobian,dynCompJacobian);
    }
}

void testRegressorConsistency(iDynTree::HighLevel::DynamicsComputations & dynComp,
                              iCub::iDynTree::DynTree & dynTree)
{
    size_t dofs = dynComp.getNrOfDegreesOfFreedom();
    size_t nrOfLinks = dynComp.getNrOfLinks();

    // check regressor matrix
    MatrixDynSize dynTreeRegressor(6+dofs,10*nrOfLinks),dynCompRegressor(6+dofs,10*nrOfLinks);
    yarp::sig::Matrix        dynTreeRegressorYarp(6+dofs,10*nrOfLinks);
    dynTree.kinematicRNEA();
    dynTree.getDynamicsRegressor(dynTreeRegressorYarp);
    yarp2idyntree(dynTreeRegressorYarp,dynTreeRegressor);

    dynComp.getDynamicsRegressor(dynCompRegressor);

    ASSERT_EQUAL_MATRIX(dynTreeRegressor,dynCompRegressor);

    // check parameter vector
    VectorDynSize dynTreeParams(10*nrOfLinks),dynCompParams(10*nrOfLinks);
    yarp::sig::Vector        dynTreeParamsYarp(10*nrOfLinks);
    dynTree.getDynamicsParameters(dynTreeParamsYarp);
    yarp2idyntree(dynTreeParamsYarp,dynTreeParams);
    dynComp.getModelDynamicsParameters(dynCompParams);

    ASSERT_EQUAL_VECTOR(dynTreeParams,dynCompParams);
}

void testCOMConsistency(iDynTree::HighLevel::DynamicsComputations & dynComp,
                        iCub::iDynTree::DynTree & dynTree)
{
    size_t dofs = dynComp.getNrOfDegreesOfFreedom();

    // check COM position
    iDynTree::Position COMnew, COMold;

    yarp::sig::Vector COMYarp;
    COMYarp = dynTree.getCOM();

    toiDynTree(COMYarp,COMold);

    COMnew = dynComp.getCenterOfMass();

    ASSERT_EQUAL_VECTOR(COMnew,COMold);

    // check COM Jacobian
    iDynTree::MatrixDynSize jacNew(3,6+dofs), jacOld(3,6+dofs);
    yarp::sig::Matrix jacYARP(6,6+dofs);

    bool ok = dynTree.getCOMJacobian(jacYARP);

    ASSERT_IS_TRUE(ok);

    iDynTree::toEigen(jacOld) = iDynTree::toEigen(jacYARP).topRows<3>();

    ok = dynComp.getCenterOfMassJacobian(jacNew);

    ASSERT_IS_TRUE(ok);

    std::cerr << "jacNew \n " << jacNew.toString() << std::endl;

    std::cerr << "jacOld \n " << jacOld.toString() << std::endl;

    ASSERT_EQUAL_MATRIX(jacNew,jacOld);
}


void assertConsistency(std::string modelName)
{
    std::cerr << "DynamicsComputationsDynTreeConsistencyUnitTest running on model "
              << modelName << std::endl;
    iDynTree::HighLevel::DynamicsComputations dynComp;
    iCub::iDynTree::DynTree dynTree;

    bool ok = dynComp.loadRobotModelFromFile(modelName);
         ok = ok && dynTree.loadURDFModel(modelName);

    ASSERT_IS_TRUE(ok);

    ASSERT_EQUAL_DOUBLE(dynTree.getNrOfDOFs(),dynComp.getNrOfDegreesOfFreedom());
    ASSERT_EQUAL_DOUBLE(dynTree.getNrOfFrames(),dynComp.getNrOfFrames());
    
    // Load model with the same joint ordering of the DynTree 
    iDynTree::ModelLoader mdlLoader;
    ok = mdlLoader.loadModelFromFile(modelName);
    ASSERT_IS_TRUE(ok);
    
    std::cerr << "Loaded model " << std::endl;
    
    std::vector<std::string> consideredJoints;
    KDL::CoDyCo::UndirectedTree undirectedTree = dynTree.getKDLUndirectedTree();
    for(int i=0; i < undirectedTree.getNrOfJunctions(); i++)
    {
        std::string jointName = undirectedTree.getJunction(i)->getName();
        
        // If the joint belong to the new model, include it in the considered joints 
        if( mdlLoader.model().isJointNameUsed(jointName) )
        {
            consideredJoints.push_back(jointName);
        }
    }
    
    iDynTree::ModelLoader mdlLoaderReduced;
    ok = mdlLoaderReduced.loadReducedModelFromFullModel(mdlLoader.model(),consideredJoints);
    ASSERT_IS_TRUE(ok);
    
    iDynTree::KinDynComputations kinDynComp;
    
    ok = kinDynComp.loadRobotModel(mdlLoaderReduced.model());
    ASSERT_IS_TRUE(ok);
    
    std::cerr << "Loaded model in kinDynComp" << std::endl;

    
    setRandomState(dynComp,dynTree,kinDynComp);
    std::cerr << "Test transforms " << std::endl;
    testTransformsConsistency(dynComp,dynTree,kinDynComp);
    std::cerr << "Test jacob " << std::endl;
    testJacobianConsistency(dynComp,dynTree,kinDynComp);
    testRegressorConsistency(dynComp,dynTree);
    testCOMConsistency(dynComp,dynTree);
}

int main()
{
    assertConsistency(getAbsModelPath("oneLink.urdf"));
    assertConsistency(getAbsModelPath("twoLinks.urdf"));
    assertConsistency(getAbsModelPath("icub.urdf"));

    return EXIT_SUCCESS;
}
