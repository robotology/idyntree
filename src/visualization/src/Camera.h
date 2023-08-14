// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_CAMERA_H
#define IDYNTREE_CAMERA_H

#include <iDynTree/Visualizer.h>
#include "CameraAnimator.h"
#include <irrlicht.h>

namespace iDynTree
{

class Camera: public ICamera
{
private:
    irr::scene::ICameraSceneNode * m_irrCamera;
    CameraAnimator* m_animator;

public:
    Camera();
    virtual ~Camera();

    void setIrrlichtCamera(irr::scene::ICameraSceneNode * cam);

    void setCameraAnimator(CameraAnimator* animator);

    irr::scene::ICameraSceneNode * irrlichtCamera();

    void setAspectRatio(double aspectRatio);

    void setWindowDimensions(unsigned int width, unsigned int height);

    virtual void setPosition(const Position& cameraPos);
    virtual void setTarget(const Position& targetPos);
    virtual void setUpVector(const Direction& upVector);
    virtual iDynTree::Position getPosition();
    virtual iDynTree::Position getTarget();
    virtual ICameraAnimator* animator();
};

}

#endif
