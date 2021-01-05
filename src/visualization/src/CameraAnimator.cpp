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

namespace iDynTree
{

//! constructor
CameraAnimator::CameraAnimator(irr::gui::ICursorControl* cursor,
    irr::f32 rotateSpeed, irr::f32 zoomSpeed, irr::f32 translateSpeed, irr::f32 distance)
    : m_cursorControl(cursor), m_oldCamera(0), m_mousePos(0.5f, 0.5f), m_initialMousePosition(m_mousePos),
    m_zoomSpeed(zoomSpeed), m_rotateSpeed(rotateSpeed), m_translateSpeed(translateSpeed),
    m_currentZoom(distance), m_rotX(0.0f), m_rotY(0.0f),
    m_zooming(false), m_rotating(false), m_moving(false), m_translating(false)
{
    #ifdef _DEBUG
    setDebugName("CSceneNodeAnimatorCameraMaya");
    #endif

    if (m_cursorControl)
    {
        m_cursorControl->grab();
        m_mousePos = m_cursorControl->getRelativePosition();
    }

    allKeysUp();
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
void CameraAnimator::animateNode(irr::scene::ISceneNode *node, irr::u32 timeMs)
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

    irr::core::vector3df initialTarget = camera->getTarget();
    irr::core::vector3df newTarget = initialTarget;

    irr::core::vector3df initialPosition = camera->getPosition();
    irr::core::vector3df newPosition = initialPosition;

    irr::core::matrix4 initialTransformation = camera->getAbsoluteTransformation();

    irr::core::vector2df mouseDelta = m_mousePos - m_initialMousePosition;

    if (m_wheelMoving)
    {
        m_wheelMoving = false;
        m_zooming = true;
        m_currentZoom = m_wheelDirection;
    }

    if (m_mouseKeys[2])
    {
        m_translating = true;
        m_initialMousePosition = m_mousePos;
    }

    if (m_zooming)
    {
        m_zooming = false;
        irr::f32 distanceFromInitialPosition = m_currentZoom * m_zoomSpeed;
        irr::f32 distanceFromTarget = newPosition.getDistanceFrom(newTarget);
        irr::f32 minimumDistance = 0.01;

        if (distanceFromTarget - distanceFromInitialPosition < minimumDistance)
        {
            distanceFromInitialPosition = distanceFromTarget - minimumDistance;
        }

        irr::f32 interpolationValue = distanceFromInitialPosition / newPosition.getDistanceFrom(newTarget);

        newPosition += interpolationValue * (newTarget - newPosition);
    }

    if (m_translating)
    {
        m_translating = false;
        irr::core::vector3df deltaInCameraCoordinates(m_translateSpeed * mouseDelta.Y, m_translateSpeed * mouseDelta.X, 0.0);
        irr::core::vector3df deltaInWorld;
        initialTransformation.rotateVect(deltaInWorld, deltaInCameraCoordinates);
        newPosition += deltaInWorld;
        newTarget += deltaInWorld;
    }

    camera->setPosition(newPosition);
    camera->setTarget(newTarget);

//    if (m_oldCamera != camera)
//    {
//        m_lastCameraTarget = m_oldTarget = camera->getTarget();
//        m_oldCamera = camera;
//    }
//    else
//    {
//        m_oldTarget += camera->getTarget() - m_lastCameraTarget;
//    }

//    irr::f32 nRotX = m_rotX;
//    irr::f32 nRotY = m_rotY;
//    irr::f32 nZoom = m_currentZoom;

//    if ( (isMouseKeyDown(0) && isMouseKeyDown(2)) || isMouseKeyDown(1) )
//    {
//        if (!m_zooming)
//        {
//            m_zoomStart = m_mousePos;
//            m_zooming = true;
//        }
//        else
//        {
//            const irr::f32 targetMinDistance = 0.1f;
//            nZoom += (m_zoomStart.X - m_mousePos.X) * m_zoomSpeed;

//            if (nZoom < targetMinDistance) // jox: fixed bug: bounce back when zooming to close
//                nZoom = targetMinDistance;
//        }
//    }
//    else if (m_zooming)
//    {
//        const irr::f32 old = m_currentZoom;
//        m_currentZoom = m_currentZoom + (m_zoomStart.X - m_mousePos.X ) * m_zoomSpeed;
//        nZoom = m_currentZoom;

//        if (nZoom < 0)
//            nZoom = m_currentZoom = old;
//        m_zooming = false;
//    }

//    // Translation ---------------------------------

//    irr::core::vector3df translate(m_oldTarget);
//    const irr::core::vector3df upVector(camera->getUpVector());
//    const irr::core::vector3df target = camera->getTarget();

//    irr::core::vector3df pos = camera->getPosition();
//    irr::core::vector3df tvectX = pos - target;
//    tvectX = tvectX.crossProduct(upVector);
//    tvectX.normalize();

//    const irr::scene::SViewFrustum* const va = camera->getViewFrustum();
//    irr::core::vector3df tvectY = (va->getFarLeftDown() - va->getFarRightDown());
//    tvectY = tvectY.crossProduct(upVector.Y > 0 ? pos - target : target - pos);
//    tvectY.normalize();

//    if (isMouseKeyDown(2) && !m_zooming)
//    {
//        if (!m_translating)
//        {
//            m_translateStart = m_mousePos;
//            m_translating = true;
//        }
//        else
//        {
//            translate +=  tvectX * (m_translateStart.X - m_mousePos.X)*m_translateSpeed +
//                          tvectY * (m_translateStart.Y - m_mousePos.Y)*m_translateSpeed;
//        }
//    }
//    else if (m_translating)
//    {
//        translate += tvectX * (m_translateStart.X - m_mousePos.X)*m_translateSpeed +
//                     tvectY * (m_translateStart.Y - m_mousePos.Y)*m_translateSpeed;
//        m_oldTarget = translate;
//        m_translating = false;
//    }

//    // Rotation ------------------------------------

//    if (isMouseKeyDown(0) && !m_zooming)
//    {
//        if (!m_rotating)
//        {
//            m_rotateStart = m_mousePos;
//            m_rotating = true;
//            nRotX = m_rotX;
//            nRotY = m_rotY;
//        }
//        else
//        {
//            nRotX += (m_rotateStart.X - m_mousePos.X) * m_rotateSpeed;
//            nRotY += (m_rotateStart.Y - m_mousePos.Y) * m_rotateSpeed;
//        }
//    }
//    else if (m_rotating)
//    {
//        m_rotX += (m_rotateStart.X - m_mousePos.X) * m_rotateSpeed;
//        m_rotY += (m_rotateStart.Y - m_mousePos.Y) * m_rotateSpeed;
//        nRotX = m_rotX;
//        nRotY = m_rotY;
//        m_rotating = false;
//    }

//    // Set pos ------------------------------------

//    pos = translate;
//    pos.X += nZoom;

//    pos.rotateXYBy(nRotY, translate);
//    pos.rotateXZBy(-nRotX, translate);

//    camera->setPosition(pos);
//    camera->setTarget(translate);

//    // Rotation Error ----------------------------

//    // jox: fixed bug: jitter when rotating to the top and bottom of y
//    pos.set(0,1,0);
//    pos.rotateXYBy(-nRotY);
//    pos.rotateXZBy(-nRotX+180.f);
//    camera->setUpVector(pos);
//    m_lastCameraTarget = camera->getTarget();
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
void CameraAnimator::setRotateSpeed(irr::f32 speed)
{
    m_rotateSpeed = speed;
}


//! Sets the movement speed
void CameraAnimator::setMoveSpeed(irr::f32 speed)
{
    m_translateSpeed = speed;
}


//! Sets the zoom speed
void CameraAnimator::setZoomSpeed(irr::f32 speed)
{
    m_zoomSpeed = speed;
}


//! Set the distance
void CameraAnimator::setDistance(irr::f32 distance)
{
    m_currentZoom=distance;
}


//! Gets the rotation speed
irr::f32 CameraAnimator::getRotateSpeed() const
{
    return m_rotateSpeed;
}


// Gets the movement speed
irr::f32 CameraAnimator::getMoveSpeed() const
{
    return m_translateSpeed;
}


//! Gets the zoom speed
irr::f32 CameraAnimator::getZoomSpeed() const
{
    return m_zoomSpeed;
}


//! Returns the current distance, i.e. orbit radius
irr::f32 CameraAnimator::getDistance() const
{
    return m_currentZoom;
}


irr::scene::ISceneNodeAnimator* CameraAnimator::createClone(irr::scene::ISceneNode* node, irr::scene::ISceneManager* newManager)
{
    CameraAnimator * newAnimator =
        new CameraAnimator(m_cursorControl, m_rotateSpeed, m_zoomSpeed, m_translateSpeed);
    return newAnimator;
}

} // end namespace

