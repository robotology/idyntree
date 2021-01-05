/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_CAMERAANIMATOR_H
#define IDYNTREE_CAMERAANIMATOR_H

#include <irrlicht.h>

namespace iDynTree
{
    //! Special scene node animator for Maya-style cameras
    /** This scene node animator can be attached to a camera to make it act like a 3d
    modelling tool.
    The camera is moving relative to the target with the mouse, by pressing either
    of the three buttons.
    In order to move the camera, set a new target for the camera. The distance defines
    the current orbit radius the camera moves on. Distance can be changed via the setter
    or by mouse events.
    */
    class ICameraAnimator : public irr::scene::ISceneNodeAnimator
    {
    public:

        //! Returns the speed of movement
        virtual irr::f32 getMoveSpeed() const = 0;

        //! Sets the speed of movement
        virtual void setMoveSpeed(irr::f32 moveSpeed) = 0;

        //! Returns the rotation speed
        virtual irr::f32 getRotateSpeed() const = 0;

        //! Set the rotation speed
        virtual void setRotateSpeed(irr::f32 rotateSpeed) = 0;

        //! Returns the zoom speed
        virtual irr::f32 getZoomSpeed() const = 0;

        //! Set the zoom speed
        virtual void setZoomSpeed(irr::f32 zoomSpeed) = 0;

        //! Returns the current distance, i.e. orbit radius
        virtual irr::f32 getDistance() const = 0;

        //! Set the distance
        virtual void setDistance(irr::f32 distance) = 0;
    };
} // end namespace irr

namespace iDynTree
{

    //! Special scene node animator for FPS cameras
    /** This scene node animator can be attached to a camera to make it act
    like a 3d modelling tool camera
    */
    class CameraAnimator : public ICameraAnimator
    {
    public:
        //! Constructor
        CameraAnimator(irr::gui::ICursorControl* cursor, irr::f32 rotateSpeed = -1500.f,
            irr::f32 zoomSpeed = 1.f, irr::f32 translationSpeed = 10.0f, irr::f32 distance=70.f);

        //! Destructor
        virtual ~CameraAnimator();

        //! Animates the scene node, currently only works on cameras
        virtual void animateNode(irr::scene::ISceneNode* node, irr::u32 timeMs);

        //! Event receiver
        virtual bool OnEvent(const irr::SEvent& event);

        //! Returns the speed of movement in units per millisecond
        virtual irr::f32 getMoveSpeed() const;

        //! Sets the speed of movement in units per millisecond
        virtual void setMoveSpeed(irr::f32 moveSpeed);

        //! Returns the rotation speed
        virtual irr::f32 getRotateSpeed() const;

        //! Set the rotation speed
        virtual void setRotateSpeed(irr::f32 rotateSpeed);

        //! Returns the zoom speed
        virtual irr::f32 getZoomSpeed() const;

        //! Set the zoom speed
        virtual void setZoomSpeed(irr::f32 zoomSpeed);

        //! Returns the current distance, i.e. orbit radius
        virtual irr::f32 getDistance() const;

        //! Set the distance
        virtual void setDistance(irr::f32 distance);

        //! This animator will receive events when attached to the active camera
        virtual bool isEventReceiverEnabled() const
        {
            return true;
        }

        //! Returns type of the scene node
        virtual irr::scene::ESCENE_NODE_ANIMATOR_TYPE getType() const
        {
            return irr::scene::ESNAT_CAMERA_MAYA;
        }

        //! Creates a clone of this animator.
        /** Please note that you will have to drop
        (IReferenceCounted::drop()) the returned pointer after calling
        this. */
        virtual irr::scene::ISceneNodeAnimator* createClone(irr::scene::ISceneNode* node, irr::scene::ISceneManager* newManager=0);

    private:

        void allKeysUp();
        void animate();
        bool isMouseKeyDown(irr::s32 key) const;

        bool m_mouseKeys[3];
        bool m_wheelMoving;
        irr::f32 m_wheelDirection;

        irr::gui::ICursorControl *m_cursorControl;
        irr::scene::ICameraSceneNode* m_oldCamera;
        irr::core::vector3df m_oldTarget;
        irr::core::vector3df m_lastCameraTarget;    // to find out if the camera target was moved outside this animator
        irr::core::position2df m_rotateStart;
        irr::core::position2df m_zoomStart;
        irr::core::position2df m_translateStart;
        irr::core::position2df m_mousePos;
        irr::core::position2df m_initialMousePosition;
        irr::f32 m_zoomSpeed;
        irr::f32 m_rotateSpeed;
        irr::f32 m_translateSpeed;
        irr::f32 m_currentZoom;
        irr::f32 m_rotX, m_rotY;
        bool m_zooming;
        bool m_rotating;
        bool m_moving;
        bool m_translating;
    };

} // end namespace irr


#endif // IDYNTREE_CAMERAANIMATOR_H
