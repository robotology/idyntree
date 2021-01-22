/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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

#include <cmath>

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
    double kRad2deg = 180/M_PI;
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
    double kDeg2rad = M_PI/180;
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

inline irr::video::SMaterial idyntree2irr(const iDynTree::ColorViz & rgbaMaterial)
{
    iDynTree::Vector4 vec4;
    vec4(0) = rgbaMaterial.r;
    vec4(1) = rgbaMaterial.g;
    vec4(2) = rgbaMaterial.b;
    vec4(3) = rgbaMaterial.a;
    return idyntree2irr(vec4);
}

inline iDynTree::Transform irr2idyntree_trans(const irr::core::matrix4 & transIrr)
{
    iDynTree::Matrix4x4 trans;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            trans(i, j) = transIrr(i, j);
        }
    }
    return iDynTree::Transform(trans);
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

        irr::scene::IMesh* boxMesh = smgr->getGeometryCreator()->createCubeMesh(irr::core::vector3df(box->getX(),box->getY(),box->getZ()));

        geomNode = smgr->addMeshSceneNode(boxMesh,linkNode);

        boxMesh->drop();

    }

    if (geom->isSphere())
    {
        const iDynTree::Sphere* sphere = geom->asSphere();

        //geomNode = smgr->addSphereSceneNode(sphere->radius,16,linkNode);

        irr::scene::IMesh* sphereMesh = smgr->getGeometryCreator()->createSphereMesh(sphere->getRadius());

        geomNode = smgr->addMeshSceneNode(sphereMesh,linkNode);
    }

    if (geom->isCylinder())
    {
        const iDynTree::Cylinder* cylinder = geom->asCylinder();

        irr::scene::IMesh* cylinderMesh = smgr->getGeometryCreator()->createCylinderMesh(cylinder->getRadius(),cylinder->getLength(),16);

        // The Irrlicht geometry creator creates meshes that are symmetric wrt to the y axis and the origin in the base
        // while iDynTree/URDF has them parallel to the z axis, and the origin in the center
        // so we rotate and translate the mesh before creating the scene node
        irr::core::matrix4 irr2idyntree;
        irr2idyntree.buildRotateFromTo(irr::core::vector3df(0.0,1.0,0.0),irr::core::vector3df(0.0,0.0,1.0));
        irr2idyntree(3,2) = -cylinder->getLength()/2.0;

        smgr->getMeshManipulator()->transform(cylinderMesh,irr2idyntree);

        geomNode = smgr->addMeshSceneNode(cylinderMesh,linkNode);
    }

    if (geom->isExternalMesh())
    {
        const iDynTree::ExternalMesh* externalMesh = geom->asExternalMesh();

        irr::scene::IAnimatedMesh* loadedAnimatedMesh = smgr->getMesh(externalMesh->getFileLocationOnLocalFileSystem().c_str());

        if (!loadedAnimatedMesh)
        {
            std::cerr << "Error in loading mesh " << externalMesh->getFileLocationOnLocalFileSystem() << std::endl;
            return 0;
        }

        // If multiple mesh are loaded, add them
        if (getFileExt(externalMesh->getFilename()) == "dae")
        {
            use_iDynTree_material = false;
        }

        iDynTree::Vector3 scale = externalMesh->getScale();

        // If multiple mesh are loaded, add them
        if (getFileExt(externalMesh->getFilename()) == "stl")
        {
            scale(0) = -scale(0); //STL meshes are interpreted as left handed by irrlicht
        }

        geomNode = smgr->addMeshSceneNode(loadedAnimatedMesh->getMesh(0),linkNode);
        geomNode->setScale(idyntree2irr_pos(scale));
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
            geomNode->getMaterial(mat) = idyntree2irr(geom->getMaterial().color());
        }
    }

    geomNode->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, false);
    geomNode->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
    geomNode->setPosition(idyntree2irr_pos(geom->getLink_H_geometry().getPosition()));
    geomNode->setRotation(idyntree2irr_rot(geom->getLink_H_geometry().getRotation()));

    return geomNode;
}

inline irr::scene::ISceneNode * addFrameAxes(irr::scene::ISceneManager* smgr,
                                             irr::scene::ISceneNode * parentNode=0,
                                             irr::f32 arrowLength=1.0)
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

    irr::f32 arrowHeight = 1.2*arrowLength;
    irr::f32 cylinderHeight = arrowLength;
    irr::f32 cylinderWidth = std::max(0.01*arrowLength, 0.005);
    irr::f32 coneWidth = 5 * cylinderWidth;

    irr::scene::IMesh* arrowMesh = smgr->getGeometryCreator()->createArrowMesh(4,8, arrowHeight, cylinderHeight, cylinderWidth, coneWidth);

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

    arrowMesh->drop();

    return frameNode;
}

inline CFloorGridSceneNode * addFloorGridNode(irr::scene::ISceneManager* smgr,
                                                irr::scene::ISceneNode * parentNode=0)
{
    return new CFloorGridSceneNode(parentNode,smgr);
}

inline void setWorldHNode(irr::scene::ISceneNode* node, const iDynTree::Transform & trans)
{
    node->setPosition(idyntree2irr_pos(trans.getPosition()));
    node->setRotation(idyntree2irr_rot(trans.getRotation()));
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
    camera->bindTargetAndRotation(true); //To change the target and the rotation at the same time

    // See http://irrlicht.sourceforge.net/forum/viewtopic.php?f=4&t=47734
    irr::core::matrix4 matproj = camera->getProjectionMatrix();
    matproj(0,0) *= -1;
    camera->setProjectionMatrix(matproj);

    return camera;
}

// Imported from http://irrlicht.sourceforge.net/forum/viewtopic.php?f=9&t=45413
// TODO : rewrite for efficency
inline irr::scene::IMesh* createFrustumMesh(irr::f32 topRadius,
                                            irr::f32 bottomRadius,
                                            irr::f32 height,
                                            irr::u32 tesselation = 8)
{
    irr::scene::SMeshBuffer* buffer = new irr::scene::SMeshBuffer();
    irr::video::S3DVertex v;
    irr::f32 radius;
    irr::f32 angle;
    irr::f32 x, y, z;
    irr::f32 s, t;

    for (irr::u32 m = 0; m <= tesselation; ++m) // height step:
    {
        z = height * irr::f32(m) / irr::f32(tesselation);
        radius = ((height - z) * (bottomRadius - topRadius) / height) + topRadius;
        //radius = 15.0f;
        t = 1.0f - irr::f32(m) / irr::f32(tesselation);
        for (irr::u32 n = 0; n <= tesselation; ++n) // subdivide circle:
        {
            angle = irr::core::PI * 2.0f * irr::f32(n) / irr::f32(tesselation);
            x = radius * cosf(angle);
            y = radius * sinf(angle);
            s = irr::f32(n) / irr::f32(tesselation);

            v.Pos.X = x;
            v.Pos.Y = y;
            v.Pos.Z = z;

            v.Normal = v.Pos;
            v.Normal.normalize();
            v.Color = 0xFFFFFFFF;

            //v.TCoords.X = s;
            //v.TCoords.Y = t;
            v.TCoords = irr::core::vector2df(s, t);

            buffer->Vertices.push_back(v);
            //printf("(s,t)=%f,%f\n", s, t);
        }
    }

    irr::u32 index00, index10, index11, index01;
    for (irr::u32 m = 0; m < tesselation; ++m)
    {
        for (irr::u32 n = 0; n < tesselation; ++n)
        {
            index00 = m * (tesselation + 1) + n;
            index10 = (m + 1) * (tesselation + 1) + n;
            index11 = (m + 1) * (tesselation + 1) + n + 1;
            index01 = m * (tesselation + 1) + n + 1;

            buffer->Indices.push_back(index00);
            buffer->Indices.push_back(index10);
            buffer->Indices.push_back(index11);

            buffer->Indices.push_back(index00);
            buffer->Indices.push_back(index11);
            buffer->Indices.push_back(index01);
        }
    }

    buffer->recalculateBoundingBox();
    irr::scene::SMesh* mesh = new irr::scene::SMesh();
    mesh->addMeshBuffer(buffer);
    buffer->drop();

    mesh->setHardwareMappingHint(irr::scene::EHM_STATIC);
    mesh->recalculateBoundingBox();
    return mesh;
}



}

#endif
