/*
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
#ifndef IDYNTREE_FRAMEVISUALIZATION_H
#define IDYNTREE_FRAMEVISUALIZATION_H

#include <iDynTree/Visualizer.h>

#include <vector>
#include <irrlicht.h>

namespace iDynTree
{
    class FrameVisualization : public IFrameVisualization
    {
        struct Frame
        {
            irr::scene::ISceneNode * visualizationNode = nullptr;
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

    };
}

#endif // IDYNTREE_FRAMEVISUALIZATION_H
