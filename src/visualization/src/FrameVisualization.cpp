// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "FrameVisualization.h"
#include "IrrlichtUtils.h"

#include <iDynTree/Model.h>


void iDynTree::FrameVisualization::setFrameTransform(size_t index, const iDynTree::Transform &transformation)
{
    irr::scene::ISceneNode * frameSceneNode = m_frames[index].visualizationNode;
    frameSceneNode->setPosition(idyntree2irr_pos(transformation.getPosition()));
    frameSceneNode->setRotation(idyntree2irr_rot(transformation.getRotation()));
}

iDynTree::FrameVisualization::FrameVisualization()
    : m_smgr(0)
{

}

size_t iDynTree::FrameVisualization::addFrame(const iDynTree::Transform &transformation, double arrowLength)
{
    m_frames.emplace_back();

    m_frames.back().visualizationNode = addFrameAxes(m_smgr, 0, arrowLength);
    m_frames.back().visualizationNode->grab();
    setFrameTransform(m_frames.size() - 1, transformation);
    m_frames.back().label.init(m_smgr, m_frames.back().visualizationNode);

    return m_frames.size() - 1;
}

bool iDynTree::FrameVisualization::setVisible(size_t frameIndex, bool isVisible)
{
    if (frameIndex >= m_frames.size()) {
        reportError("FrameVisualization","setVisible","frameIndex out of bounds.");
        return false;
    }

    m_frames[frameIndex].visualizationNode->setVisible(isVisible);

    return true;
}

size_t iDynTree::FrameVisualization::getNrOfFrames() const
{
    return m_frames.size();
}

bool iDynTree::FrameVisualization::getFrameTransform(size_t frameIndex, iDynTree::Transform &currentTransform) const
{
    if (frameIndex >= m_frames.size()) {
        reportError("FrameVisualization","getFrameTransform","frameIndex out of bounds.");
        return false;
    }

    irr::scene::ISceneNode * frameSceneNode = m_frames[frameIndex].visualizationNode;
    currentTransform.setPosition(irr2idyntree_pos(frameSceneNode->getPosition()));
    currentTransform.setRotation(irr2idyntree_rot(frameSceneNode->getRotation()));

    return true;
}

bool iDynTree::FrameVisualization::updateFrame(size_t frameIndex, const iDynTree::Transform &transformation)
{
    if (frameIndex >= m_frames.size()) {
        reportError("FrameVisualization","updateFrame","frameIndex out of bounds.");
        return false;
    }
    setFrameTransform(frameIndex, transformation);
    return true;
}

std::pair<std::string, std::string> iDynTree::FrameVisualization::getFrameParent(size_t frameIndex) const
{
    if (frameIndex >= m_frames.size())
    {
        reportError("FrameVisualization", "getFrameParent", "Frame index out of range");
        return std::make_pair<std::string, std::string>("", "");
    }
    return std::make_pair(m_frames[frameIndex].parentModel, m_frames[frameIndex].parentFrame);
}

bool iDynTree::FrameVisualization::setFrameParent(size_t frameIndex, const std::string& modelName, const std::string& frameName)
{
    if (frameIndex >= m_frames.size())
    {
        reportError("FrameVisualization", "setFrameParent", "Frame index out of range");
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
            reportError("FrameVisualization", "setFrameParent", error.c_str());
            return false;
        }
    }

    m_frames[frameIndex].visualizationNode->setParent(parent);
    m_frames[frameIndex].parentModel = modelName;
    m_frames[frameIndex].parentFrame = actualFrameName;

    return true;
}

iDynTree::ILabel *iDynTree::FrameVisualization::getFrameLabel(size_t frameIndex)
{
    if (frameIndex >= m_frames.size()) {
        reportError("FrameVisualization","getFrameLabel","frameIndex out of bounds.");
        return nullptr;
    }

    return &m_frames[frameIndex].label;
}

iDynTree::FrameVisualization::~FrameVisualization()
{
    close();
}

void iDynTree::FrameVisualization::init(irr::scene::ISceneManager* smgr, std::shared_ptr<std::vector<ModelVisualization*>> models)
{
    assert(smgr);
    m_smgr = smgr;
    m_smgr->grab(); //Increment the reference count
    m_models = models;
}

void iDynTree::FrameVisualization::close()
{
    for (auto& frames: m_frames) {
        if (frames.visualizationNode) {
            frames.visualizationNode->removeAll();
            frames.visualizationNode->remove();
            frames.visualizationNode->drop();
            frames.visualizationNode = nullptr;
        }
    }
    m_frames.resize(0);

    if (m_smgr)
    {
        m_smgr->drop(); //Drop the element (dual of "grab")
        m_smgr = 0;
    }
    m_models = nullptr;
}
