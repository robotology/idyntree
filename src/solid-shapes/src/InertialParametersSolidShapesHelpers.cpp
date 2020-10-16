/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/InertialParametersSolidShapesHelpers.h>

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
    double boxVolume = box.getX() * box.getY() * box.getZ();
    double boxMass   = density*boxVolume;
    // Assuming uniform density, the center of mass is coincident with the box center
    PositionRaw comInGeomFrame;
    comInGeomFrame.zero();
    // From http://scienceworld.wolfram.com/physics/MomentofInertiaRectangularParallelepiped.html
    RotationalInertiaRaw rotInertiaInGeomFrame;
    rotInertiaInGeomFrame.zero();
    double x2 = box.getX() * box.getX();
    double y2 = box.getY() * box.getY();
    double z2 = box.getZ() * box.getZ();
    rotInertiaInGeomFrame(0, 0) = (boxMass/12.0)*(y2+z2);
    rotInertiaInGeomFrame(1, 1) = (boxMass/12.0)*(x2+z2);
    rotInertiaInGeomFrame(2, 2) = (boxMass/12.0)*(x2+y2);

    SpatialInertia inertiaInGeometryFrame = SpatialInertia(boxMass, comInGeomFrame, rotInertiaInGeomFrame);

    return box.getLink_H_geometry() * inertiaInGeometryFrame;
}

double boxGetVolume(const Box& box)
{
    return box.getX() * box.getY() * box.getX();
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

/**
 * Convert std::vector<aiVector3D> to std::vector<iDynTree::Position>
 */
std::vector<Position> toiDynTree(std::vector<aiVector3D> assimpPoints)
{
    std::vector<Position> ret;

    for (auto&& assimpPoint : assimpPoints) {
        ret.push_back(Position(assimpPoint.x, assimpPoint.y, assimpPoint.z));
    }

    return ret;
}

Box extractAABBFromVertices(const Transform& link_H_vertices,
                        const std::vector<Position>& vertexVector)
{
    Box box;

    double minX = vertexVector[0](0);
    double maxX = vertexVector[0](0);
    double minY = vertexVector[0](1);
    double maxY = vertexVector[0](1);
    double minZ = vertexVector[0](2);
    double maxZ = vertexVector[0](2);

    for(size_t i=0; i < vertexVector.size(); i++)
    {
        if (vertexVector[i](0) > maxX)
        {
            maxX = vertexVector[i](0);
        }

        if (vertexVector[i](0) < minX)
        {
            minX = vertexVector[i](0);
        }

        if (vertexVector[i](1) > maxY)
        {
            maxY = vertexVector[i](1);
        }

        if (vertexVector[i](1) < minY)
        {
            minY = vertexVector[i](1);
        }

        if (vertexVector[i](2) > maxZ)
        {
            maxZ = vertexVector[i](2);
        }

        if (vertexVector[i](2) < minZ)
        {
            minZ = vertexVector[i](2);
        }
    }

    // The side of the BB is the difference between the max and min
    box.setX(maxX-minX);
    box.setY(maxY-minY);
    box.setZ(maxZ-minZ);

    // The offset between the geometry origin and the bounding box origin is the middle point
    Position offset_bb_wrt_geom;
    offset_bb_wrt_geom(0) = (maxX+minX)/2.0;
    offset_bb_wrt_geom(1) = (maxY+minY)/2.0;
    offset_bb_wrt_geom(2) = (maxZ+minZ)/2.0;

    Position offset_bb_wrt_link = link_H_vertices*offset_bb_wrt_geom;

    box.setLink_H_geometry(Transform(link_H_vertices.getRotation(),
                                     offset_bb_wrt_link));

    return box;
}

bool BBFromExternalShape(ExternalMesh* extMesh, Box& box)
{
    // Load mesh with assimp
    Assimp::Importer Importer;

    std::string filename = extMesh->getFilename();
    const aiScene* pScene = Importer.ReadFile(filename.c_str(), 0);

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

        // Apply scale attribute for external meshes
        double scalingFactorX=extMesh->getScale().getVal(0);
        double scalingFactorY=extMesh->getScale().getVal(1);
        double scalingFactorZ=extMesh->getScale().getVal(2);

        // Use each component to scale each vertex coordinate
        for(size_t i=0; i < vertexVector.size(); i++)
        {
            vertexVector[i].x*=scalingFactorX;
            vertexVector[i].y*=scalingFactorY;
            vertexVector[i].z*=scalingFactorZ;
        }

        box = extractAABBFromVertices(extMesh->getLink_H_geometry(), toiDynTree(vertexVector));

        return true;
    }
    else
    {
        std::stringstream ss;
        ss << "Impossible to load mesh " << extMesh->getFilename() << " using the Assimp library.";
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
        box.setLink_H_geometry(geom->getLink_H_geometry());
        return true;
    }

    if (geom->isSphere())
    {
        // If shape is a sphere all the side of the BB are the diameter of the sphere
        Sphere* pSphere = static_cast<Sphere*>(geom);
        box.setX(2.0*pSphere->getRadius());
        box.setY(2.0*pSphere->getRadius());
        box.setZ(2.0*pSphere->getRadius());
        box.setLink_H_geometry(geom->getLink_H_geometry());
        return true;
    }

    if (geom->isCylinder())
    {
        // If shape is a cylinder the x and y side of the BB are the diameter of the cylinder,
        // while the z side is the lenght of the cylinder
        Cylinder* pCylinder = static_cast<Cylinder*>(geom);
        box.setX(2.0*pCylinder->getRadius());
        box.setY(2.0*pCylinder->getRadius());
        box.setZ(pCylinder->getLength());
        box.setLink_H_geometry(geom->getLink_H_geometry());
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

/**
 * Compute vertices of the bounding box, computed in link frame
 */
std::vector<Position> computeBoxVertices(const Box box)
{
    std::vector<Position> vertices;

    // + + +
    vertices.push_back(box.getLink_H_geometry()*Position(+box.getX()/2, +box.getY()/2, +box.getZ()/2));

    // + + -
    vertices.push_back(box.getLink_H_geometry()*Position(+box.getX()/2, +box.getY()/2, -box.getZ()/2));

    // + - +
    vertices.push_back(box.getLink_H_geometry()*Position(+box.getX()/2, -box.getY()/2, +box.getZ()/2));

    // + - -
    vertices.push_back(box.getLink_H_geometry()*Position(+box.getX()/2, -box.getY()/2, -box.getZ()/2));

    // - + +
    vertices.push_back(box.getLink_H_geometry()*Position(-box.getX()/2, +box.getY()/2, +box.getZ()/2));

    // - + -
    vertices.push_back(box.getLink_H_geometry()*Position(-box.getX()/2, +box.getY()/2, -box.getZ()/2));

    // - - +
    vertices.push_back(box.getLink_H_geometry()*Position(-box.getX()/2, -box.getY()/2, +box.getZ()/2));

    // - - -
    vertices.push_back(box.getLink_H_geometry()*Position(-box.getX()/2, -box.getY()/2, -box.getZ()/2));

    return vertices;
}

/**
 * Compute the axis aligned bounding box out of a vector of axis aligned bounding boxes.
 */
Box computeAABoundingBox(const std::vector<Box>& shapesBBsInLinkFrame)
{
    // First compute the total set of vertices
    std::vector<Position> BBVertices;
    for (auto&& singleShapeBB : shapesBBsInLinkFrame) {
        std::vector<Position> singleShapeBBVertices = computeBoxVertices(singleShapeBB);

        BBVertices.insert(BBVertices.end(),
                          singleShapeBBVertices.begin(),
                          singleShapeBBVertices.end());
    }

    // Then extract the AABB of the combined vertices
    return extractAABBFromVertices(Transform::Identity(), BBVertices);
}


bool getBoundingBoxOfLinkGeometries(iDynTree::Model& model,
                                    std::vector<iDynTree::Box>& linkBBsInLinkFrame)
{
    linkBBsInLinkFrame.resize(model.getNrOfLinks());

    for (LinkIndex lnkIdx=0; lnkIdx < model.getNrOfLinks(); lnkIdx++)
    {
        // If models has no shape associated
        if (model.collisionSolidShapes().getLinkSolidShapes()[lnkIdx].size() == 0)
        {
            Box box;
            box.setX(0);
            box.setY(0);
            box.setZ(0);
            linkBBsInLinkFrame[lnkIdx] = box;
            continue;
        }

        // Bounding boxes for each shape of the link, each expressed in link frame
        std::vector<iDynTree::Box> shapesBBsInLinkFrame;
        for(int shapeIdx=0; shapeIdx < model.collisionSolidShapes().getLinkSolidShapes()[lnkIdx].size(); shapeIdx++)
        {
            iDynTree::Box shapeBoundingBox;
            SolidShape * shape = model.collisionSolidShapes().getLinkSolidShapes()[lnkIdx][shapeIdx];
            bool ok = BBFromShape(shape, shapeBoundingBox);
            if (!ok) return false;
            shapesBBsInLinkFrame.push_back(shapeBoundingBox);
        }

        // Compute resulting AABB
        linkBBsInLinkFrame[lnkIdx] = computeAABoundingBox(shapesBBsInLinkFrame);
    }

    return true;
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
    return false;
#endif
}

}
