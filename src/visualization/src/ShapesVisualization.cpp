// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
#include "ShapesVisualization.h"
#include "IrrlichtUtils.h"


iDynTree::ShapeVisualization::Shape::Shape(const SolidShape& input_shape, irr::scene::ISceneNode* parent, irr::scene::ISceneManager* sceneManager)
{
    shape.reset(input_shape.clone());
    node = addGeometryToSceneManager(shape.get(), parent, sceneManager);
    label.init(sceneManager, node);
}

iDynTree::ShapeVisualization::Shape::Shape(Shape&& other)
{
    operator=(std::move(other));
}

iDynTree::ShapeVisualization::Shape& iDynTree::ShapeVisualization::Shape::operator=(Shape&& other)
{
    if (node)
    {
        node->remove();
        node->drop();
    }
    node = other.node;
    other.node = nullptr;
    shape = std::move(other.shape);
    label = std::move(other.label);
    return *this;
}

iDynTree::ShapeVisualization::Shape::~Shape()
{
    if (node)
    {
        node->remove();
        node->drop();
    }
}

void iDynTree::ShapeVisualization::init(irr::scene::ISceneManager* smgr, std::shared_ptr<std::vector<ModelVisualization*>> models)
{
    assert(smgr);
    m_smgr = smgr;
    m_smgr->grab(); //Increment the reference count
    m_models = models;
}

void iDynTree::ShapeVisualization::close()
{
    m_shapes.clear();
    if (m_smgr)
    {
        m_smgr->drop(); //Decrement the reference count
        m_smgr = nullptr;
    }
}

iDynTree::ShapeVisualization::~ShapeVisualization()
{
    close();
}

size_t iDynTree::ShapeVisualization::addShape(const iDynTree::SolidShape& shape,const std::string& modelName, const std::string& frameName)
{
    irr::scene::ISceneNode* parent = nullptr;

    if (!modelName.empty() && !frameName.empty())
    {
        bool found = false;
        for (auto& model : *m_models)
        {
            if (model->getInstanceName() == modelName)
            {
                found = true;
                parent = model->getFrameSceneNode(frameName);
                break;
            }
        }
        if (!parent)
        {
            std::string error;
            if (!found)
            {
                error = "Model " + modelName + " not found";
            }
            else
            {
                error = "Frame " + frameName + " not found in model " + modelName;
            }
            reportError("ShapesVisualization", "addShape", error.c_str());
            return -1;
        }

    }

    m_shapes.emplace_back(shape, m_smgr->getRootSceneNode(), m_smgr);
    return m_shapes.size() - 1;
}

bool iDynTree::ShapeVisualization::setVisible(size_t shapeIndex, bool isVisible)
{
    if (shapeIndex >= m_shapes.size())
    {
        reportError("ShapesVisualization", "setVisible", "Shape index out of range");
        return false;
    }
    m_shapes[shapeIndex].node->setVisible(isVisible);
    return true;
}

size_t iDynTree::ShapeVisualization::getNrOfShapes() const
{
    return m_shapes.size();
}

bool iDynTree::ShapeVisualization::getShapeTransform(size_t shapeIndex, Transform& currentTransform) const
{
    if (shapeIndex >= m_shapes.size())
    {
        reportError("ShapesVisualization", "getShapeTransform", "Shape index out of range");
        return false;
    }
    currentTransform = m_shapes[shapeIndex].shape->getLink_H_geometry();
    return true;
}

bool iDynTree::ShapeVisualization::setShapeTransform(size_t shapeIndex, const Transform& transformation)
{
    if (shapeIndex >= m_shapes.size())
    {
        reportError("ShapesVisualization", "setShapeTransform", "Shape index out of range");
        return false;
    }
    m_shapes[shapeIndex].shape->setLink_H_geometry(transformation);
    setWorldHNode(m_shapes[shapeIndex].node, transformation);

    return true;
}

bool iDynTree::ShapeVisualization::setShapeColor(size_t shapeIndex, const ColorViz& shapeColor)
{
    if (shapeIndex >= m_shapes.size())
    {
        reportError("ShapesVisualization", "setShapeColor", "Shape index out of range");
        return false;
    }
    iDynTree::Material newMaterial = m_shapes[shapeIndex].shape->getMaterial();
    newMaterial.setColor(shapeColor.toVector4());
    m_shapes[shapeIndex].shape->setMaterial(newMaterial);
    for (irr::u32 mat = 0; mat < m_shapes[shapeIndex].node->getMaterialCount(); mat++)
    {
        m_shapes[shapeIndex].node->getMaterial(mat) = idyntree2irr(m_shapes[shapeIndex].shape->getMaterial().color());
    }
    return true;
}

bool iDynTree::ShapeVisualization::changeShape(size_t shapeIndex, const iDynTree::SolidShape& newShape)
{
    if (shapeIndex >= m_shapes.size())
    {
        reportError("ShapesVisualization", "changeShape", "Shape index out of range");
        return false;
    }
    m_shapes[shapeIndex] = std::move(Shape(newShape, m_shapes[shapeIndex].node->getParent(), m_smgr));
    return true;
}

iDynTree::ILabel* iDynTree::ShapeVisualization::getShapeLabel(size_t shapeIndex)
{
    if (shapeIndex >= m_shapes.size())
    {
        reportError("ShapesVisualization", "getShapeLabel", "Shape index out of range");
        return nullptr;
    }
    return &m_shapes[shapeIndex].label;
}
