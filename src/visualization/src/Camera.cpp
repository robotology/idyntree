/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "Camera.h"
#include "IrrlichtUtils.h"

namespace iDynTree
{

Camera::Camera(): m_irrCamera(0)
{

}

Camera::~Camera()
{
}

irr::scene::ICameraSceneNode* Camera::irrlichtCamera()
{
    return m_irrCamera;
}

void Camera::setIrrlichtCamera(irr::scene::ICameraSceneNode* cam)
{
    m_irrCamera = cam;
}

void Camera::setPosition(const Position& cameraPos)
{
    if(m_irrCamera)
    {
        m_irrCamera->setPosition(idyntree2irr_pos(cameraPos));
    }
    else
    {
        reportError("Camera","setPosition","Impossible to set position of a null camera");
    }
}

void Camera::setTarget(const Position& cameraTargetPos)
{
    if(m_irrCamera)
    {
        m_irrCamera->setTarget(idyntree2irr_pos(cameraTargetPos));
    }
    else
    {
        reportError("Camera","setTarget","Impossible to set target position of a null camera");
    }
}

}
