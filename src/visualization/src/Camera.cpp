/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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

void Camera::setUpVector(const Direction& cameraUpVector)
{
    if(m_irrCamera)
    {
        m_irrCamera->setUpVector(idyntree2irr_pos(cameraUpVector));
    }
    else
    {
        reportError("Camera","setTarget","Impossible to set up vector of a null camera");
    }
}

}
