/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <irrlicht.h>
#include "CameraAnimator.h"
#include "IrrlichtUtils.h"

namespace iDynTree
{

//! constructor
CameraAnimator::CameraAnimator(irr::gui::ICursorControl* cursor, irr::scene::ISceneNode *cameraAxis,
    double rotateSpeed, double zoomSpeed, double translateSpeed)
    : m_cursorControl(cursor), m_mousePos(0.5f, 0.5f), m_initialMousePosition(m_mousePos),
    m_zoomSpeed(zoomSpeed), m_rotateSpeed(rotateSpeed), m_translateSpeed(translateSpeed),
    m_zooming(false), m_rotating(false), m_movingUp(false), m_translating(false), m_isEnabled(false)
{
    #ifdef _DEBUG
    setDebugName("iDynTreeCameraAnimator");
    #endif

    if (m_cursorControl)
    {
        m_cursorControl->grab();
        m_mousePos = m_cursorControl->getRelativePosition();
    }

    allKeysUp();
    m_cameraAxis = cameraAxis;

}


//! destructor
CameraAnimator::~CameraAnimator()
{
    if (m_cursorControl)
        m_cursorControl->drop();
}


//! It is possible to send mouse and key events to the camera. Most cameras
//! may ignore this input, but camera scene nodes which are created for
//! example with scene::ISceneManager::addMayaCameraSceneNode or
//! scene::ISceneManager::addMeshViewerCameraSceneNode, may want to get this input
//! for changing their position, look at target or whatever.
bool CameraAnimator::OnEvent(const irr::SEvent& event)
{
    if (event.EventType != irr::EET_MOUSE_INPUT_EVENT)
        return false;

    switch(event.MouseInput.Event)
    {
    case irr::EMIE_LMOUSE_PRESSED_DOWN:
        m_mouseKeys[0] = true;
        m_initialMousePosition = m_cursorControl->getRelativePosition();
        break;
    case irr::EMIE_RMOUSE_PRESSED_DOWN:
        m_mouseKeys[2] = true;
        m_initialMousePosition = m_cursorControl->getRelativePosition();
        break;
    case irr::EMIE_MMOUSE_PRESSED_DOWN:
        m_mouseKeys[1] = true;
        break;
    case irr::EMIE_LMOUSE_LEFT_UP:
        m_mouseKeys[0] = false;
        break;
    case irr::EMIE_RMOUSE_LEFT_UP:
        m_mouseKeys[2] = false;
        break;
    case irr::EMIE_MMOUSE_LEFT_UP:
        m_mouseKeys[1] = false;
        break;
    case irr::EMIE_MOUSE_MOVED:
        m_mousePos = m_cursorControl->getRelativePosition();
        break;
    case irr::EMIE_MOUSE_WHEEL:
        m_wheelMoving = true;
        m_wheelDirection = event.MouseInput.Wheel;
        break;

    case irr::EMIE_LMOUSE_DOUBLE_CLICK:
    case irr::EMIE_RMOUSE_DOUBLE_CLICK:
    case irr::EMIE_MMOUSE_DOUBLE_CLICK:
    case irr::EMIE_LMOUSE_TRIPLE_CLICK:
    case irr::EMIE_RMOUSE_TRIPLE_CLICK:
    case irr::EMIE_MMOUSE_TRIPLE_CLICK:
    case irr::EMIE_COUNT:
        return false;
    }
    return true;
}


//! OnAnimate() is called just before rendering the whole scene.
void CameraAnimator::animateNode(irr::scene::ISceneNode *node, irr::u32 /*timeMs*/)
{
    if (!node || node->getType() != irr::scene::ESNT_CAMERA)
        return;

    irr::scene::ICameraSceneNode* camera = static_cast<irr::scene::ICameraSceneNode*>(node);

    // If the camera isn't the active camera, and receiving input, then don't process it.
    if (!camera->isInputReceiverEnabled())
        return;

    irr::scene::ISceneManager * smgr = camera->getSceneManager();
    if (smgr && smgr->getActiveCamera() != camera)
        return;

    irr::core::vector2df mouseDelta = m_mousePos - m_initialMousePosition;

    if (m_wheelMoving)
    {
        m_wheelMoving = false;
        m_zooming = true;
    }

    if (m_mouseKeys[0])
    {
        m_rotating = true;
        m_initialMousePosition = m_mousePos;
    }

    if (m_mouseKeys[1])
    {
        m_movingUp = true;
        m_initialMousePosition = m_mousePos;
    }

    if (m_mouseKeys[2])
    {
        m_translating = true;
        m_initialMousePosition = m_mousePos;
    }

    if (!m_zooming && !m_translating && !m_rotating && !m_movingUp) //Doing nothing
    {
        m_cameraAxis->setVisible(false);
        return;
    }

    irr::core::vector3df initialTarget = camera->getTarget();
    irr::core::vector3df newTarget = initialTarget;

    irr::core::vector3df initialPosition = camera->getPosition();
    irr::core::vector3df newPosition = initialPosition;
    irr::core::vector3df upVector = camera->getUpVector();

    const irr::scene::SViewFrustum* const va = camera->getViewFrustum();

    irr::core::matrix4 initialTransformation;
    initialTransformation.setTranslation(initialPosition);

    irr::core::vector3df xAxis = va->getFarLeftUp() - va->getFarLeftDown();
    xAxis.normalize();
    irr::core::vector3df yAxis;

    irr::core::vector3df zAxis = initialTarget - initialPosition;
    zAxis.normalize();

    yAxis = zAxis.crossProduct(xAxis);
    yAxis.normalize();

    // Getting the rotation directly from camera results in undesired rotations when getting close to the singularity
    // due to the fact that irrlicht stores rotations using RPY. As a consequence, we reconstruct the camera frame by
    // having the z-axis parallel to the line connecting the camera position to the target position. The x-axis is
    // reconstructed from the view frustrum. The view frustrum is the region of space that appears in the camera. It
    // is basically a truncated rectangular pyramid. It is delimited by 8 points. We use the bottom left and top
    // left points in the "far" plane to define the up direction, the x-axis. The y-axis is obtained by cross-product
    // between the other two axes, obtaining a right-handed frame. Since irrlicht uses left-handed frames, the obtained
    // frame is converted into a left-handed frame by taking its transpose (a left hand rotation corresponds to the
    // inverse of a right hand rotation since the axis is the same, but the angle is on the opposite direction).

    initialTransformation(0,0) = xAxis.X;
    initialTransformation(0,1) = xAxis.Y;
    initialTransformation(0,2) = xAxis.Z;

    initialTransformation(1,0) = yAxis.X;
    initialTransformation(1,1) = yAxis.Y;
    initialTransformation(1,2) = yAxis.Z;

    initialTransformation(2,0) = zAxis.X;
    initialTransformation(2,1) = zAxis.Y;
    initialTransformation(2,2) = zAxis.Z;

    m_cameraAxis->setPosition(initialTarget);
    m_cameraAxis->setRotation(initialTransformation.getRotationDegrees());
    m_cameraAxis->setVisible(true);

    irr::f32 minimumDistanceFromTarget = 0.01;

    if (m_zooming)
    {
        m_zooming = false;
        irr::f32 distanceFromInitialPosition = m_wheelDirection * m_zoomSpeed;
        irr::f32 distanceFromTarget = newPosition.getDistanceFrom(newTarget);

        if (distanceFromTarget - distanceFromInitialPosition < minimumDistanceFromTarget)
        {
            distanceFromInitialPosition = distanceFromTarget - minimumDistanceFromTarget;
        }

        irr::f32 interpolationValue = distanceFromInitialPosition / newPosition.getDistanceFrom(newTarget);

        newPosition += interpolationValue * (newTarget - newPosition);
    }

    if (m_translating)
    {
        m_translating = false;
        irr::core::vector3df deltaInCameraCoordinates(m_translateSpeed * mouseDelta.Y, -m_translateSpeed * mouseDelta.X, 0.0);
        irr::core::vector3df deltaInWorld;
        initialTransformation.rotateVect(deltaInWorld, deltaInCameraCoordinates);
        irr::f32 translation = deltaInWorld.getLength(); //Save the translation amount

        if (translation > 0.01)
        {
            //Remove up movements
            deltaInWorld = deltaInWorld - deltaInWorld.dotProduct(upVector) * upVector;

            //Restore translation amount
            deltaInWorld.setLength(translation);
            newPosition += deltaInWorld;
            newTarget += deltaInWorld;
        }
    }

    if (m_rotating)
    {
        m_rotating = false;
        irr::f32 initialDistance = newPosition.getDistanceFrom(newTarget);
        irr::core::vector3df deltaInCameraCoordinates(m_rotateSpeed * mouseDelta.Y, -m_rotateSpeed * mouseDelta.X, 0.0);
        irr::core::vector3df deltaInWorld;
        initialTransformation.rotateVect(deltaInWorld, deltaInCameraCoordinates);
        irr::core::vector3df desiredPosition = newPosition + deltaInWorld;

        irr::core::vector3df  differenceFromTarget = desiredPosition - newTarget;
        differenceFromTarget.setLength(initialDistance); //Reset distance from target

        newPosition = newTarget + differenceFromTarget;
    }

    if (m_movingUp)
    {
        m_movingUp = false;
        irr::f32 delta = mouseDelta.Y * m_translateSpeed;
        newPosition.Z += delta;
        newTarget.Z += delta;
    }

    camera->setPosition(newPosition);
    camera->setTarget(newTarget);

}


bool CameraAnimator::isMouseKeyDown(irr::s32 key) const
{
    return m_mouseKeys[key];
}


void CameraAnimator::allKeysUp()
{
    for (irr::s32 i=0; i<3; ++i)
        m_mouseKeys[i] = false;

    m_wheelMoving = false;
    m_wheelDirection = 0;
}


//! Sets the rotation speed
void CameraAnimator::setRotateSpeed(double speed)
{
    m_rotateSpeed = speed;
}


//! Sets the movement speed
void CameraAnimator::setMoveSpeed(double speed)
{
    m_translateSpeed = speed;
}


//! Sets the zoom speed
void CameraAnimator::setZoomSpeed(double speed)
{
    m_zoomSpeed = speed;
}

bool CameraAnimator::isEventReceiverEnabled() const
{
    return m_isEnabled;
}


//! Gets the rotation speed
double CameraAnimator::getRotateSpeed() const
{
    return m_rotateSpeed;
}


// Gets the movement speed
double CameraAnimator::getMoveSpeed() const
{
    return m_translateSpeed;
}


//! Gets the zoom speed
double CameraAnimator::getZoomSpeed() const
{
    return m_zoomSpeed;
}


irr::scene::ISceneNodeAnimator* CameraAnimator::createClone(irr::scene::ISceneNode* /*node*/, irr::scene::ISceneManager* /*newManager*/)
{
    CameraAnimator * newAnimator =
        new CameraAnimator(m_cursorControl, m_cameraAxis->clone(), m_rotateSpeed, m_zoomSpeed, m_translateSpeed);
    return newAnimator;
}

void CameraAnimator::enableMouseControl(bool enable)
{
    m_isEnabled = enable;
}

} // end namespace

