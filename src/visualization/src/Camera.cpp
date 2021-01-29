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

Camera::Camera(): m_irrCamera(0), m_animator(0)
{

}

Camera::~Camera()
{
    if (m_animator)
    {
        m_animator->drop();
        m_animator = 0;
    }
}

irr::scene::ICameraSceneNode* Camera::irrlichtCamera()
{
    return m_irrCamera;
}

void Camera::setAspectRatio(double aspectRatio)
{
    if(m_irrCamera)
    {
        m_irrCamera->setAspectRatio(aspectRatio);

        //Setting the aspect ratio seems to reset the visualization of left handed frames (hence everything appears mirrored)
        // See http://irrlicht.sourceforge.net/forum/viewtopic.php?f=4&t=47734
        irr::core::matrix4 matproj = m_irrCamera->getProjectionMatrix();
        matproj(0,0) *= -1;
        m_irrCamera->setProjectionMatrix(matproj);
    }
    else
    {
        reportError("Camera","setAspectRatio","Impossible to set the aspect ratio of a null camera");
    }
}

void Camera::setIrrlichtCamera(irr::scene::ICameraSceneNode* cam)
{
    m_irrCamera = cam;
}

void Camera::setCameraAnimator(CameraAnimator *animator)
{
    if(m_irrCamera)
    {
        m_animator = animator;
        m_irrCamera->addAnimator(m_animator);
    }
    else
    {
        reportError("Camera","setCameraAnimator","Impossible to set the animator of a null camera");
    }
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

Position Camera::getPosition()
{
    if(m_irrCamera)
    {
        return irr2idyntree_pos(m_irrCamera->getPosition());
    }
    else
    {
        reportError("Camera","getPosition","Impossible to get the position of a null camera");
        return iDynTree::Position();
    }
}

Position Camera::getTarget()
{
    if(m_irrCamera)
    {
        return irr2idyntree_pos(m_irrCamera->getTarget());
    }
    else
    {
        reportError("Camera","getTarget","Impossible to get the target of a null camera");
        return iDynTree::Position();
    }
}

ICameraAnimator *Camera::animator()
{
    if (!m_animator)
    {
        reportError("Camera","animator","The visualizer has not been initialed yet.");
    }
    return m_animator;
}

}
