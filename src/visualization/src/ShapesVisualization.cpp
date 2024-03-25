// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
#include "ShapesVisualization.h"
#include "IrrlichtUtils.h"

#include <iDynTree/Model.h>


iDynTree::ShapeVisualization::Shape::Shape(const SolidShape& input_shape, irr::scene::ISceneManager* sceneManager)
{
    shape.reset(input_shape.clone());
    node = addGeometryToSceneManager(shape.get(), nullptr, sceneManager);
    node->grab(); //Increment the reference count, otherwise it can be deleted when setting the parent
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
        node->drop(); //Decrement the reference count
    }
    node = other.node;
    other.node = nullptr;
    shape = std::move(other.shape);
    label = std::move(other.label);
    modelName = std::move(other.modelName);
    frameName = std::move(other.frameName);
    return *this;
}

iDynTree::ShapeVisualization::Shape::~Shape()
{
    if (node)
    {
        node->remove();
        node->drop(); //Decrement the reference count
        node = nullptr;
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
    m_models = nullptr;
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
    m_shapes.emplace_back(shape, m_smgr);
    setShapeParent(m_shapes.size() - 1, modelName, frameName);
    setShapeColor(m_shapes.size() - 1, shape.getMaterial().color());
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
        irr::video::SMaterial& material = m_shapes[shapeIndex].node->getMaterial(mat);
        material = idyntree2irr(m_shapes[shapeIndex].shape->getMaterial().color());
        if (shapeColor.a < 1.0)
        {
            material.MaterialType = irr::video::EMT_TRANSPARENT_VERTEX_ALPHA;
        }
        else
        {
            material.MaterialType = irr::video::EMT_SOLID;
        }
    }
    m_shapes[shapeIndex].node->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, false);
    m_shapes[shapeIndex].node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
    m_shapes[shapeIndex].node->setMaterialFlag(irr::video::EMF_COLOR_MATERIAL, false);
    m_shapes[shapeIndex].node->setMaterialFlag(irr::video::EMF_BLEND_OPERATION, true);
    return true;
}

bool iDynTree::ShapeVisualization::changeShape(size_t shapeIndex, const iDynTree::SolidShape& newShape)
{
    if (shapeIndex >= m_shapes.size())
    {
        reportError("ShapesVisualization", "changeShape", "Shape index out of range");
        return false;
    }
    m_shapes[shapeIndex] = std::move(Shape(newShape, m_smgr));
    setShapeColor(shapeIndex, newShape.getMaterial().color());
    return true;
}

std::pair<std::string, std::string> iDynTree::ShapeVisualization::getShapeParent(size_t shapeIndex) const
{
    if (shapeIndex >= m_shapes.size())
    {
        reportError("ShapesVisualization", "getShapeParent", "Shape index out of range");
        return std::make_pair<std::string, std::string>("", "");
    }
    return std::make_pair(m_shapes[shapeIndex].modelName, m_shapes[shapeIndex].frameName);
}

bool iDynTree::ShapeVisualization::setShapeParent(size_t shapeIndex, const std::string& modelName, const std::string& frameName)
{
    if (shapeIndex >= m_shapes.size())
    {
        reportError("ShapesVisualization", "setShapeParent", "Shape index out of range");
        return false;
    }

    irr::scene::ISceneNode* parent = nullptr;
    std::string actualFrameName = "";

    if (!modelName.empty())
    {
        bool found = false;
        for (auto& model : *m_models)
        {
            if (model->getInstanceName() == modelName)
            {
                found = true;
                if (frameName.empty())
                {
                    iDynTree::LinkIndex root_link_index = model->model().getDefaultBaseLink();
                    actualFrameName = model->model().getLinkName(root_link_index);
                    parent = model->getFrameSceneNode(actualFrameName);
                }
                else
                {
                    actualFrameName = frameName;
                    parent = model->getFrameSceneNode(frameName);
                }
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
            reportError("ShapesVisualization", "setShapeParent", error.c_str());
            return false;
        }
    }

    m_shapes[shapeIndex].node->setParent(parent);
    m_shapes[shapeIndex].modelName = modelName;
    m_shapes[shapeIndex].frameName = actualFrameName;

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
