/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

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

    virtual void setPosition(const Position& cameraPos);
    virtual void setTarget(const Position& targetPos);
    virtual void setUpVector(const Direction& upVector);
    virtual iDynTree::Position getPosition();
    virtual iDynTree::Position getTarget();
    virtual ICameraAnimator* animator();
};

}

#endif
