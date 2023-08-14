// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
#ifndef IDYNTREE_FRAMEVISUALIZATION_H
#define IDYNTREE_FRAMEVISUALIZATION_H

#include <iDynTree/Visualizer.h>
#include "Label.h"

#include <vector>
#include <irrlicht.h>

namespace iDynTree
{
    class FrameVisualization : public IFrameVisualization
    {
        struct Frame
        {
            irr::scene::ISceneNode * visualizationNode = nullptr;
            Label label;
        };

        std::vector<Frame> m_frames;
        irr::scene::ISceneManager* m_smgr;

        void setFrameTransform(size_t index, const Transform& transformation);

    public:

        FrameVisualization();

        ~FrameVisualization();

        void init(irr::scene::ISceneManager* smgr);

        void close();

        FrameVisualization(const FrameVisualization& other) = delete;

        FrameVisualization& operator=(const FrameVisualization& other) = delete;

        virtual size_t addFrame(const Transform& transformation, double arrowLength = 1.0) final;

        virtual bool setVisible(size_t frameIndex, bool isVisible) final;

        virtual size_t getNrOfFrames() const final;

        virtual bool getFrameTransform(size_t frameIndex, Transform& currentTransform) const final;

        virtual bool updateFrame(size_t frameIndex, const Transform& transformation) final;

        virtual ILabel* getFrameLabel(size_t frameIndex) final;

    };
}

#endif // IDYNTREE_FRAMEVISUALIZATION_H
