/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_CAMERA_H
#define IDYNTREE_CAMERA_H

#include <iDynTree/Visualizer.h>

#include <irrlicht.h>

namespace iDynTree
{

class Camera: public ICamera
{
private:
    irr::scene::ICameraSceneNode * m_irrCamera;

public:
    Camera();
    virtual ~Camera();

    void setIrrlichtCamera(irr::scene::ICameraSceneNode * cam);

    irr::scene::ICameraSceneNode * irrlichtCamera();

    virtual void setPosition(const Position& cameraPos);
    virtual void setTarget(const Position& cameraPos);
};

}

#endif
