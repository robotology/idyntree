/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Model/InertialParametersHelpers.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/SpatialInertia.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>

#include <iDynTree/Model/SolidShapes.h>

#ifdef IDYNTREE_USES_ASSIMP
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#endif

namespace iDynTree
{

// Helper methods for the box shape
// This should be eventually be moved to be methods of the Box class
SpatialInertia boxGet6DInertiaInLinkFrameFromDensity(const Box& box,
                                                     double density)
{
    std::cerr << "boxGet6DInertiaInLinkFrameFromDensity " << box.x << " " << box.y << " " << box.z << std::endl;

    double boxVolume = box.x*box.y*box.z;
    double boxMass   = density*boxVolume;
    // Assuming uniform density, the center of mass is coincident with the box center
    PositionRaw comInGeomFrame;
    comInGeomFrame.zero();
    // From http://scienceworld.wolfram.com/physics/MomentofInertiaRectangularParallelepiped.html
    RotationalInertiaRaw rotInertiaInGeomFrame;
    rotInertiaInGeomFrame.zero();
    double x2 = box.x*box.x;
    double y2 = box.y*box.y;
    double z2 = box.z*box.z;
    rotInertiaInGeomFrame(0, 0) = (boxMass/12.0)*(y2+z2);
    rotInertiaInGeomFrame(1, 1) = (boxMass/12.0)*(x2+z2);
    rotInertiaInGeomFrame(2, 2) = (boxMass/12.0)*(x2+y2);

    SpatialInertia inertiaInGeometryFrame = SpatialInertia(boxMass, comInGeomFrame, rotInertiaInGeomFrame);

    std::cerr << "boxGet6DInertiaInLinkFrameFromDensity " << inertiaInGeometryFrame.getRotationalInertiaWrtFrameOrigin().toString() << std::endl;
    std::cerr << "boxGet6DInertiaInLinkFrameFromDensity " << (box.link_H_geometry*inertiaInGeometryFrame).getRotationalInertiaWrtFrameOrigin().toString() << std::endl;


    return box.link_H_geometry*inertiaInGeometryFrame;
}

double boxGetVolume(const Box& box)
{
    return box.x*box.y*box.z;
}

#ifdef IDYNTREE_USES_ASSIMP

// Inspired  from https://github.com/ros-visualization/rviz/blob/070835c426b8982e304b38eb4a9c6eb221155d5f/src/rviz/mesh_loader.cpp#L644
void buildMesh(const aiScene* scene, const aiNode* node, const double scale, std::vector<aiVector3D>& vertexVector)
{
  if (!node)
  {
    return;
  }

  aiMatrix4x4 transform = node->mTransformation;
  aiNode *pnode = node->mParent;
  while (pnode)
  {
    // Don't convert to y-up orientation, which is what the root node in
    // Assimp does
    if (pnode->mParent != NULL)
      transform = pnode->mTransformation * transform;
    pnode = pnode->mParent;
  }

  aiMatrix3x3 rotation(transform);
  aiMatrix3x3 inverse_transpose_rotation(rotation);
  inverse_transpose_rotation.Inverse();
  inverse_transpose_rotation.Transpose();

  for (uint32_t i = 0; i < node->mNumMeshes; i++)
  {
    aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];

    // Add the vertices
    for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)
    {
      aiVector3D p = input_mesh->mVertices[j];
      p *= transform;
      p *= scale;
      vertexVector.push_back(p);
    }
  }

  for (uint32_t i=0; i < node->mNumChildren; ++i)
  {
    buildMesh(scene, node->mChildren[i], scale, vertexVector);
  }
}


bool BBFromExternalShape(ExternalMesh* extMesh, Box& box)
{
    // Load mesh with assimp
    Assimp::Importer Importer;

    const aiScene* pScene = Importer.ReadFile(extMesh->filename.c_str(), 0);

    if (pScene)
    {
        // Extract vector of vertices
        double scale = 1.0;
        std::vector<aiVector3D> vertexVector;
        buildMesh(pScene, pScene->mRootNode, scale, vertexVector);

        if (vertexVector.size() == 0)
        {
            return false;
        }

        double minX = vertexVector[0].x;
        double maxX = vertexVector[0].x;
        double minY = vertexVector[0].y;
        double maxY = vertexVector[0].y;
        double minZ = vertexVector[0].z;
        double maxZ = vertexVector[0].z;

        for(size_t i=0; i < vertexVector.size(); i++)
        {
            if (vertexVector[i].x > maxX)
            {
                maxX = vertexVector[i].x;
            }

            if (vertexVector[i].x < minX)
            {
                minX = vertexVector[i].x;
            }

            if (vertexVector[i].y > maxY)
            {
                maxY = vertexVector[i].y;
            }

            if (vertexVector[i].y < minY)
            {
                minY = vertexVector[i].y;
            }

            if (vertexVector[i].z > maxZ)
            {
                maxZ = vertexVector[i].z;
            }

            if (vertexVector[i].z < minZ)
            {
                minZ = vertexVector[i].z;
            }
        }

        // The side of the BB is the difference between the max and min
        box.x = maxX-minX;
        box.y = maxY-minY;
        box.z = maxZ-minZ;

        // The offset between the geometry origin and the bounding box origin is the middle point
        Position offset_bb_wrt_geom;
        offset_bb_wrt_geom(0) = (maxX+minX)/2.0;
        offset_bb_wrt_geom(1) = (maxY+minY)/2.0;
        offset_bb_wrt_geom(2) = (maxZ+minZ)/2.0;

        std::cerr << "  extMesh->link_H_geometry " <<  extMesh->link_H_geometry.toString() << std::endl;

        // Workaround for bug
        Position offset_bb_wrt_link = extMesh->link_H_geometry*offset_bb_wrt_geom;


        box.link_H_geometry = Transform(extMesh->link_H_geometry.getRotation(),
                                         offset_bb_wrt_link);


        return true;
    }
    else
    {
        std::stringstream ss;
        ss << "Impossible to load mesh " << extMesh->filename << " using the Assimp library.";
        reportError("", "BBFromExternalShape", ss.str().c_str());
        return false;
    }
}

bool BBFromShape(SolidShape* geom, Box& box)
{
    // Extract BB from shape, this would be benefic from being moved as a method in the SolidShape interface
    if (geom->isBox())
    {
        // If shape is a box, just copy it
        Box* pBox = static_cast<Box*>(geom);
        box = *pBox;
        box.link_H_geometry = geom->link_H_geometry;
        return true;
    }

    if (geom->isSphere())
    {
        // If shape is a sphere all the side of the BB are the diameter of the sphere
        Sphere* pSphere = static_cast<Sphere*>(geom);
        box.x = box.y = box.z = 2.0*pSphere->radius;
        box.link_H_geometry = geom->link_H_geometry;
        return true;
    }

    if (geom->isCylinder())
    {
        // If shape is a cylinder the x and y side of the BB are the diameter of the cylinder,
        // while the z side is the lenght of the cylinder
        Cylinder* pCylinder = static_cast<Cylinder*>(geom);
        box.x = box.y = 2.0*pCylinder->radius;
        box.z = pCylinder->length;
        box.link_H_geometry = geom->link_H_geometry;
        return true;
    }

    if (geom->isExternalMesh())
    {
        // If shape is an external mesh, we need to load the mesh and extract the BB
        ExternalMesh* pExtMesh = static_cast<ExternalMesh*>(geom);
        return BBFromExternalShape(pExtMesh, box);
    }

    return false;
}


bool getBoundingBoxOfLinkGeometries(iDynTree::Model& model,
                                    std::vector<iDynTree::Box>& linkBBsInLinkFrame)
{
    linkBBsInLinkFrame.resize(model.getNrOfLinks());

    for (LinkIndex lnkIdx=0; lnkIdx < model.getNrOfLinks(); lnkIdx++)
    {
        // We should have a for like the following, but for now we assume that there is one geometry for link
        // for(int shapeIdx=0; shapeIdx < fullModel.visualSolidShapes().linkSolidShapes[visitedLinkIndex].size(); shapeIdx++)
        if (model.visualSolidShapes().linkSolidShapes[lnkIdx].size() > 1)
        {
        //    reportError("", "boxGet6DInertiaInLinkFrameFromDensity", "Only one shape for link supported at the moment.");
        //    return false;
        }

        if (model.visualSolidShapes().linkSolidShapes[lnkIdx].size() == 0)
        {
            Box box;
            box.x = box.y = box.z = 0.0;
            linkBBsInLinkFrame[lnkIdx] = box;
        }

        //if (model.visualSolidShapes().linkSolidShapes[lnkIdx].size() == 1)
        //{
            SolidShape * shape = model.visualSolidShapes().linkSolidShapes[lnkIdx][0];
            bool ok = BBFromShape(shape, linkBBsInLinkFrame[lnkIdx]);
            if (!ok) return false;
        //}
    }
}
#endif

bool estimateInertialParametersFromLinkBoundingBoxesAndTotalMass(const double totalMass,
                                                                 iDynTree::Model& model,
                                                                 VectorDynSize& estimatedInertialParams)
{
#ifdef IDYNTREE_USES_ASSIMP
    // Resize the result vector
    const int NR_OF_INERTIAL_PARAMS_FOR_LINK = 10;
    const int nrOfInertialParametersOfModel = NR_OF_INERTIAL_PARAMS_FOR_LINK*model.getNrOfLinks();
    estimatedInertialParams.resize(nrOfInertialParametersOfModel);

    // Resize some internal buffers
    std::vector<iDynTree::Box> boundingBoxVolume(model.getNrOfLinks());
    std::vector<iDynTree::SpatialInertia> linkInertias(model.getNrOfLinks());

    // Compute the bounding box for each link
    bool ok = getBoundingBoxOfLinkGeometries(model, boundingBoxVolume);
    if (!ok) return false;

    double totalModelBBVolume = 0.0;
    for (LinkIndex lnkIdx=0; lnkIdx < model.getNrOfLinks(); lnkIdx++)
    {
        totalModelBBVolume += boxGetVolume(boundingBoxVolume[lnkIdx]);
    }

    // Compute mass for each link
    // Assume constant density for the robot
    double density = totalMass/totalModelBBVolume;
    for (LinkIndex lnkIdx=0; lnkIdx < model.getNrOfLinks(); lnkIdx++)
    {
        linkInertias[lnkIdx] = boxGet6DInertiaInLinkFrameFromDensity(boundingBoxVolume[lnkIdx], density);
    }

    // Convert inertias to vector
    for(LinkIndex linkIdx = 0; linkIdx < model.getNrOfLinks(); linkIdx++ )
    {
        Vector10       inertiaParamsBuf = linkInertias[linkIdx].asVector();
        toEigen(estimatedInertialParams).segment<10>(10*linkIdx) = toEigen(inertiaParamsBuf);
    }

    return true;
#else
    reportError("", "estimateInertialParametersFromLinkBoundingBoxesAndTotalMass", "IDYNTREE_USES_ASSIMP CMake option need to be set to ON to use estimateInertialParametersFromLinkBoundingBoxesAndTotalMass");
    return false
#endif
}

}
