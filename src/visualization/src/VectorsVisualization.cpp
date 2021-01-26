/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "VectorsVisualization.h"

#include "IrrlichtUtils.h"

#include <cassert>
#include <string>
#include <cmath>

using namespace iDynTree;



void VectorsVisualization::drawVector(size_t vectorIndex)
{
    // Delete existing node
    if (m_vectors[vectorIndex].visualizationNode)
    {
        m_vectors[vectorIndex].visualizationNode->remove();
        m_vectors[vectorIndex].visualizationNode = nullptr;
    }

    float arrowHeight = static_cast<float>(std::abs(m_vectors[vectorIndex].modulus) * m_heightScale);
    float arrowWidth = static_cast<float>(m_radiusOffset +
                                          m_radiusMultiplier * std::abs(m_vectors[vectorIndex].modulus) * m_heightScale);


    auto arrowPosition = idyntree2irr_pos(m_vectors[vectorIndex].origin);

    iDynTree::Direction vectorDirection = (m_vectors[vectorIndex].modulus < 0) ?
                m_vectors[vectorIndex].direction.reverse() :
                m_vectors[vectorIndex].direction;

    iDynTree::Direction yDirection(0.0, 1.0, 0.0); //Arrows in irrlicht are pointing in the y direction by default

    iDynTree::Direction rotationAxis;

    iDynTree::toEigen(rotationAxis) = iDynTree::toEigen(yDirection).cross(iDynTree::toEigen(vectorDirection));
    rotationAxis.Normalize();

    double rotationAngle = std::acos(iDynTree::toEigen(vectorDirection).dot(iDynTree::toEigen(yDirection)));

    iDynTree::Rotation arrowRotationMatrix = iDynTree::Rotation::RotAxis(rotationAxis, rotationAngle);

    irr::core::vector3df arrowRotation = idyntree2irr_rot(arrowRotationMatrix);

    irr::scene::ISceneNode * frameNode = m_smgr->addEmptySceneNode();

    irr::scene::IMesh* arrowMesh = m_smgr->getGeometryCreator()->createArrowMesh(4, 8, arrowHeight, 0.9f * arrowHeight,
                                                                                 arrowWidth, 2.0f * arrowWidth);

    m_vectors[vectorIndex].visualizationNode = m_smgr->addMeshSceneNode(arrowMesh,frameNode);
    m_vectors[vectorIndex].visualizationNode->setPosition(arrowPosition);
    m_vectors[vectorIndex].visualizationNode->setRotation(arrowRotation);
    irr::video::SMaterial arrowColor;
    arrowColor.AmbientColor = idyntree2irrlicht(m_vectors[vectorIndex].color).toSColor();
    arrowColor.DiffuseColor = arrowColor.AmbientColor;
    m_vectors[vectorIndex].visualizationNode->getMaterial(0) = arrowColor;
    m_vectors[vectorIndex].visualizationNode->getMaterial(1) = arrowColor;


    arrowMesh->drop();
    arrowMesh = nullptr;
}

void VectorsVisualization::drawAll()
{
    for (size_t i = 0; i < m_vectors.size(); ++i) {
        drawVector(i);
    }
}

VectorsVisualization::VectorsVisualization()
    : m_smgr(nullptr)
    , m_radiusOffset(0.01)
    , m_radiusMultiplier(0.0)
    , m_heightScale(1.0)
{

}

void VectorsVisualization::init(irr::scene::ISceneManager *smgr)
{
    assert(smgr);
    m_smgr = smgr;
    m_smgr->grab();
}

void VectorsVisualization::close()
{
    for (auto& vector: m_vectors) {
        if (vector.visualizationNode) {
            vector.visualizationNode->removeAll();
            vector.visualizationNode = nullptr;
        }
    }
    m_vectors.resize(0);

    if (m_smgr)
    {
        m_smgr->drop();
        m_smgr = nullptr;
    }
}

VectorsVisualization::~VectorsVisualization()
{
    close();
}

size_t VectorsVisualization::addVector(const Position &origin, const Direction &direction, double modulus)
{
    VectorsProperties newVector;
    newVector.origin = origin;
    newVector.direction = direction;
    newVector.modulus = modulus;

    m_vectors.push_back(newVector);

    drawVector(m_vectors.size()-1);

    return m_vectors.size() - 1;
}

size_t VectorsVisualization::addVector(const Position &origin, const Vector3 &components)
{
    return addVector(origin, Direction(components(0), components(1), components(2)), toEigen(components).norm());
}

size_t VectorsVisualization::getNrOfVectors() const
{
    return m_vectors.size();
}

bool VectorsVisualization::getVector(size_t vectorIndex, Position &currentOrigin, Direction &currentDirection, double &currentModulus) const
{
    if (vectorIndex >= m_vectors.size()) {
        reportError("VectorsVisualization","getVector","vectorIndex out of bounds.");
        return false;
    }

    currentDirection = m_vectors[vectorIndex].direction;
    currentOrigin = m_vectors[vectorIndex].origin;
    currentModulus = m_vectors[vectorIndex].modulus;

    return true;
}

bool VectorsVisualization::getVector(size_t vectorIndex, Position &currentOrigin, Vector3 &components) const
{
    if (vectorIndex >= m_vectors.size()) {
        reportError("VectorsVisualization","getVector","vectorIndex out of bounds.");
        return false;
    }

    currentOrigin = m_vectors[vectorIndex].origin;
    toEigen(components) = m_vectors[vectorIndex].modulus * toEigen(m_vectors[vectorIndex].direction);

    return true;
}

bool VectorsVisualization::updateVector(size_t vectorIndex, const Position &origin, const Direction &direction, double modulus)
{
    if (vectorIndex >= m_vectors.size()) {
        reportError("VectorsVisualization","updateVector","vectorIndex out of bounds.");
        return false;
    }

    m_vectors[vectorIndex].origin = origin;
    m_vectors[vectorIndex].direction = direction;
    m_vectors[vectorIndex].modulus = modulus;

    drawVector(vectorIndex);

    return true;
}

bool VectorsVisualization::updateVector(size_t vectorIndex, const Position &origin, const Vector3 &components)
{
    return updateVector(vectorIndex, origin, Direction(components(0), components(1), components(2)), toEigen(components).norm());
}

bool VectorsVisualization::setVectorColor(size_t vectorIndex, const ColorViz &vectorColor)
{
    if (vectorIndex >= m_vectors.size()) {
        reportError("VectorsVisualization","setVectorColor","vectorIndex out of bounds.");
        return false;
    }

    m_vectors[vectorIndex].color = vectorColor;

    drawVector(vectorIndex);

    return true;
}

bool VectorsVisualization::setVectorsAspect(double zeroModulusRadius, double modulusMultiplier, double heightScale)
{
    if (zeroModulusRadius < 0) {
        reportError("VectorsVisualization","setVectorsAspect","zeroModulusRadius is supposed to be non negative.");
        return false;
    }

    if (modulusMultiplier < 0) {
        reportError("VectorsVisualization","setVectorsAspect","modulusMultiplier is supposed to be non negative.");
        return false;
    }

    if (heightScale < 0) {
        reportError("VectorsVisualization","setVectorsAspect","heightScale is supposed to be non negative.");
        return false;
    }

    m_radiusOffset = zeroModulusRadius;
    m_radiusMultiplier = modulusMultiplier;
    m_heightScale = heightScale;

    drawAll();

    return true;
}
