/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_IRRLICHT_UTILS_H
#define IDYNTREE_IRRLICHT_UTILS_H

#include <iDynTree/Model/SolidShapes.h>
#include <iDynTree/Visualizer.h>

#include <irrlicht.h>

#include <Eigen/Core>
#include <Eigen/LU>

#include <iDynTree/Core/EigenHelpers.h>

#include "FloorGridSceneNode.h"

namespace iDynTree
{

inline irr::video::SColorf idyntree2irrlicht(iDynTree::ColorViz color)
{
    return irr::video::SColorf(color.r,color.g,color.b,color.a);
}

inline iDynTree::ColorViz  irrlicht2idyntree(irr::video::SColorf color)
{
    return iDynTree::ColorViz((double)color.r,(double)color.g,(double)color.b,(double)color.a);
}


inline irr::core::vector3df idyntree2irr_rpy(const iDynTree::Vector3 & vecId)
{
    const double kRad2deg = 180/M_PI;
    return irr::core::vector3df(kRad2deg*vecId(0),kRad2deg*vecId(1),kRad2deg*vecId(2));
}

inline irr::core::vector3df idyntree2irr_pos(const iDynTree::Vector3 & vecId)
{
    return irr::core::vector3df(vecId(0),vecId(1),vecId(2));
}

inline  iDynTree::Position irr2idyntree_pos(const irr::core::vector3df & vecIrr)
{
    return iDynTree::Position(vecIrr.X,vecIrr.Y,vecIrr.Z);
}

inline iDynTree::Vector3 irr2idyntree_rpy(const irr::core::vector3df & vecIrr)
{
    const double kDeg2rad = M_PI/180;
    iDynTree::Vector3 ret;
    ret(0) = kDeg2rad*vecIrr.X;
    ret(1) = kDeg2rad*vecIrr.Y;
    ret(2) = kDeg2rad*vecIrr.Z;
    return ret;
}

inline iDynTree::Rotation irr2idyntree_rot(const irr::core::vector3df & rotIrr)
{
    iDynTree::Vector3 rpy = irr2idyntree_rpy(rotIrr);
    return iDynTree::Rotation::RPY(rpy(0),rpy(1),rpy(2));
}

inline const irr::core::vector3df idyntree2irr_rot(const iDynTree::Rotation & rot)
{
    return idyntree2irr_rpy(rot.asRPY());
}

inline irr::video::SMaterial idyntree2irr(const iDynTree::Vector4 & rgbaMaterialId)
{
    double ambientCoeff = 1.0;
    double diffuseCoeff = 1.0;
    irr::video::SMaterial ret;
    ret.AmbientColor = irr::video::SColorf(ambientCoeff*rgbaMaterialId(0),
                                           ambientCoeff*rgbaMaterialId(1),
                                           ambientCoeff*rgbaMaterialId(2),
                                                        rgbaMaterialId(3)).toSColor();
    ret.DiffuseColor = irr::video::SColorf(diffuseCoeff*rgbaMaterialId(0),
                                           diffuseCoeff*rgbaMaterialId(1),
                                           diffuseCoeff*rgbaMaterialId(2),
                                                        rgbaMaterialId(3)).toSColor();

    return ret;
}

/**
 * Get a rotation matrix whose z column is the provided direction.
 */
inline iDynTree::Rotation RotationWithPrescribedZColumn(const iDynTree::Direction zAxis)
{
    // We need a normal vector perpendicular to z : we can extract this
    // from the nullspace of the transpose of z
    Eigen::Vector3d z = iDynTree::toEigen(zAxis);
    Eigen::Matrix<double,1,3> zTrans = z.transpose();

    // x is a normal vector orthognal to z
    Eigen::Vector3d x = zTrans.fullPivLu().kernel().block<3,1>(0,0).normalized();

    Eigen::Vector3d y = z.cross(x);

    iDynTree::Rotation R;

    toEigen(R).block<3,1>(0,0) = x;
    toEigen(R).block<3,1>(0,1) = y;
    toEigen(R).block<3,1>(0,2) = z;

    return R;
}

inline std::string getFileExt(const std::string filename)
{
    std::string::size_type idx;

    idx = filename.rfind('.');

    if (idx != std::string::npos)
    {
       return filename.substr(idx+1);
    }
    else
    {
        return "";
    }
}

inline irr::scene::ISceneNode * addGeometryToSceneManager(const iDynTree::SolidShape * geom,
                                              irr::scene::ISceneNode* linkNode,
                                              irr::scene::ISceneManager* smgr)
{
    // In general we use the material provided by iDynTree, unless the external
    // mesh has a specific formal (for now collada) that we now specifies internally materials
    bool use_iDynTree_material = true;

    irr::scene::ISceneNode * geomNode = 0;

    if (geom->isBox())
    {
        const iDynTree::Box* box = geom->asBox();

        irr::scene::IMesh* boxMesh = smgr->getGeometryCreator()->createCubeMesh(irr::core::vector3df(box->x,box->y,box->z));

        geomNode = smgr->addMeshSceneNode(boxMesh,linkNode);

    }

    if (geom->isSphere())
    {
        const iDynTree::Sphere* sphere = geom->asSphere();

        //geomNode = smgr->addSphereSceneNode(sphere->radius,16,linkNode);

        irr::scene::IMesh* sphereMesh = smgr->getGeometryCreator()->createSphereMesh(sphere->radius);

        geomNode = smgr->addMeshSceneNode(sphereMesh,linkNode);
    }

    if (geom->isCylinder())
    {
        const iDynTree::Cylinder* cylinder = geom->asCylinder();

        irr::scene::IMesh* cylinderMesh = smgr->getGeometryCreator()->createCylinderMesh(cylinder->radius,cylinder->length,16);

        // The Irrlicht geometry creator creates meshes that are symmetric wrt to the y axis and the origin in the base
        // while iDynTree/URDF has them parallel to the z axis, and the origin in the center
        // so we rotate and translate the mesh before creating the scene node
        irr::core::matrix4 irr2idyntree;
        irr2idyntree.buildRotateFromTo(irr::core::vector3df(0.0,1.0,0.0),irr::core::vector3df(0.0,0.0,1.0));
        irr2idyntree(3,2) = -cylinder->length/2.0;

        smgr->getMeshManipulator()->transform(cylinderMesh,irr2idyntree);

        geomNode = smgr->addMeshSceneNode(cylinderMesh,linkNode);
    }

    if (geom->isExternalMesh())
    {
        const iDynTree::ExternalMesh* externalMesh = geom->asExternalMesh();

        irr::scene::IAnimatedMesh* loadedAnimatedMesh = smgr->getMesh(externalMesh->filename.c_str());

        if (!loadedAnimatedMesh)
        {
            std::cerr << "Error in loading mesh " << externalMesh->filename << std::endl;
            return 0;
        }

        // If multiple mesh are loaded, add them
        if (getFileExt(externalMesh->filename) == "dae")
        {
            use_iDynTree_material = false;
        }

        geomNode = smgr->addMeshSceneNode(loadedAnimatedMesh->getMesh(0),linkNode);
        geomNode->setScale(idyntree2irr_pos(externalMesh->scale));


    }

    if (!geomNode)
    {
        std::cerr << "[ERROR] addGeometryToSceneManager: geometry not loaded" << std::endl;
        return 0;
    }

    // Set materials
    if (use_iDynTree_material)
    {
        for(irr::u32 mat=0; mat < geomNode->getMaterialCount(); mat++)
        {
            geomNode->getMaterial(mat) = idyntree2irr(geom->material);
        }
    }

    geomNode->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, false);
    geomNode->setPosition(idyntree2irr_pos(geom->link_H_geometry.getPosition()));
    geomNode->setRotation(idyntree2irr_rpy(geom->link_H_geometry.getRotation().asRPY()));

    return geomNode;
}

inline irr::scene::ISceneNode * addFrameAxes(irr::scene::ISceneManager* smgr,
                                             irr::scene::ISceneNode * parentNode=0,
                                             irr::f32 arrowLenght=1.0)
{
    irr::u32 alphaLev = 20;
    irr::video::SMaterial transRed;
    transRed.AmbientColor = irr::video::SColor(alphaLev,255,0,0);
    transRed.DiffuseColor = irr::video::SColor(alphaLev,255,0,0);

    irr::video::SMaterial transGreen;
    transGreen.AmbientColor = irr::video::SColor(alphaLev,0,255,0);
    transGreen.DiffuseColor = irr::video::SColor(alphaLev,0,255,0);

    irr::video::SMaterial transBlue;
    transBlue.AmbientColor = irr::video::SColor(alphaLev,0,0,255);
    transBlue.DiffuseColor = irr::video::SColor(alphaLev,0,0,255);

    irr::video::SMaterial transYellow;
    transYellow.AmbientColor = irr::video::SColor(alphaLev,255,255,0);
    transYellow.DiffuseColor = irr::video::SColor(alphaLev,255,255,0);

    irr::video::SMaterial transGray;
    transYellow.AmbientColor = irr::video::SColor(alphaLev,100,100,100);
    transYellow.DiffuseColor = irr::video::SColor(alphaLev,100,100,100);

    irr::scene::ISceneNode * frameNode = smgr->addEmptySceneNode(parentNode);

    irr::scene::IMesh* arrowMesh = smgr->getGeometryCreator()->createArrowMesh(4,8,1.2*arrowLenght,arrowLenght,0.01*arrowLenght,0.05*arrowLenght);

    irr::scene::ISceneNode* xArrow = smgr->addMeshSceneNode(arrowMesh,frameNode);
    xArrow->setPosition(irr::core::vector3df(0.0,0.0,0.0));
    xArrow->setRotation(irr::core::vector3df(0.0,0.0,-90.0));
    xArrow->getMaterial(0) = transRed;
    xArrow->getMaterial(1) = transRed;

    irr::scene::ISceneNode* yArrow = smgr->addMeshSceneNode(arrowMesh,frameNode);
    yArrow->setPosition(irr::core::vector3df(0.0,0.0,0.0));
    yArrow->setRotation(irr::core::vector3df(0.0,0.0,0.0));
    yArrow->getMaterial(0) = transGreen;
    yArrow->getMaterial(1) = transGreen;

    irr::scene::ISceneNode* zArrow = smgr->addMeshSceneNode(arrowMesh,frameNode);
    zArrow->setPosition(irr::core::vector3df(0.0,0.0,0.0));
    zArrow->setRotation(irr::core::vector3df(90.0,0.0,0.0));
    zArrow->getMaterial(0) = transBlue;
    zArrow->getMaterial(1) = transBlue;

    return frameNode;
}

inline irr::scene::ISceneNode * addFloorGridNode(irr::scene::ISceneManager* smgr,
                                                irr::scene::ISceneNode * parentNode=0)
{
    irr::scene::ISceneNode * retPtr;
    retPtr = new CFloorGridSceneNode(parentNode,smgr);
    return retPtr;
}

inline void setWorldHNode(irr::scene::ISceneNode* node, const iDynTree::Transform & trans)
{
    node->setPosition(idyntree2irr_pos(trans.getPosition()));
    node->setRotation(idyntree2irr_rpy(trans.getRotation().asRPY()));
}

/**
 * Add the visualization enviroment (plane, frame of ref) to the scene.
 */
void inline addVizEnviroment(irr::scene::ISceneManager* smgr)
{
    // Add origin frame
    irr::u32 alphaLev = 20;
    irr::video::SMaterial transRed;
    transRed.AmbientColor = irr::video::SColor(alphaLev,255,0,0);
    transRed.DiffuseColor = irr::video::SColor(alphaLev,255,0,0);

    irr::video::SMaterial transGreen;
    transGreen.AmbientColor = irr::video::SColor(alphaLev,0,255,0);
    transGreen.DiffuseColor = irr::video::SColor(alphaLev,0,255,0);

    irr::video::SMaterial transBlue;
    transBlue.AmbientColor = irr::video::SColor(alphaLev,0,0,255);
    transBlue.DiffuseColor = irr::video::SColor(alphaLev,0,0,255);

    irr::video::SMaterial transYellow;
    transYellow.AmbientColor = irr::video::SColor(alphaLev,255,255,0);
    transYellow.DiffuseColor = irr::video::SColor(alphaLev,255,255,0);

    irr::video::SMaterial transGray;
    transYellow.AmbientColor = irr::video::SColor(alphaLev,100,100,100);
    transYellow.DiffuseColor = irr::video::SColor(alphaLev,100,100,100);

    irr::scene::IMesh* arrowMesh = smgr->getGeometryCreator()->createArrowMesh(4,8,1.2,1.0,0.01,0.05);

    irr::scene::ISceneNode* xArrow = smgr->addMeshSceneNode(arrowMesh);
    xArrow->setPosition(irr::core::vector3df(0.0,0.0,0.0));
    xArrow->setRotation(irr::core::vector3df(0.0,0.0,-90.0));
    xArrow->getMaterial(0) = transRed;
    xArrow->getMaterial(1) = transRed;

    irr::scene::ISceneNode* yArrow = smgr->addMeshSceneNode(arrowMesh);
    yArrow->setPosition(irr::core::vector3df(0.0,0.0,0.0));
    yArrow->setRotation(irr::core::vector3df(0.0,0.0,0.0));
    yArrow->getMaterial(0) = transGreen;
    yArrow->getMaterial(1) = transGreen;

    irr::scene::ISceneNode* zArrow = smgr->addMeshSceneNode(arrowMesh);
    zArrow->setPosition(irr::core::vector3df(0.0,0.0,0.0));
    zArrow->setRotation(irr::core::vector3df(90.0,0.0,0.0));
    zArrow->getMaterial(0) = transBlue;
    zArrow->getMaterial(1) = transBlue;


}

void inline addVizLights(irr::scene::ISceneManager* smgr)
{
    smgr->setAmbientLight(irr::video::SColor(255,255,255,255));
}

inline irr::scene::ICameraSceneNode* addVizCamera(irr::scene::ISceneManager* smgr)
{
    irr::scene::ICameraSceneNode* camera = smgr->addCameraSceneNode(0,
                                                                    irr::core::vector3df(-1,0,0),
                                                                    irr::core::vector3df(1,0,0));
    //scene::ICameraSceneNode* camera = smgr->addCameraSceneNodeMaya();
    camera->setPosition(irr::core::vector3df(0.8,0.8,0.8));
    camera->setTarget(irr::core::vector3df(0,0,0));
    camera->setUpVector(irr::core::vector3df(0.0,0.0,1.0));

    // See http://irrlicht.sourceforge.net/forum/viewtopic.php?f=4&t=47734
    irr::core::matrix4 matproj = camera->getProjectionMatrix();
    matproj(0,0) *= -1;
    camera->setProjectionMatrix(matproj);

    return camera;
}


}

#endif
