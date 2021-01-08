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
#include <iDynTree/Visualizer.h>

namespace iDynTree
{

    class CameraAnimator : public ICameraAnimator, public irr::scene::ISceneNodeAnimator
    {
    public:
        //! Constructor
        CameraAnimator(irr::gui::ICursorControl* cursor, irr::scene::ISceneNode *cameraAxis, double rotateSpeed = 10.0f,
            double zoomSpeed = 0.5f, double translationSpeed = 10.0f);

        //! Destructor
        virtual ~CameraAnimator();

        //! Animates the scene node, currently only works on cameras
        virtual void animateNode(irr::scene::ISceneNode* node, irr::u32 timeMs) override;

        //! Event receiver
        virtual bool OnEvent(const irr::SEvent& event) override;

        //! Returns the speed of movement in units per millisecond
        virtual double getMoveSpeed() const override;

        //! Sets the speed of movement in units per millisecond
        virtual void setMoveSpeed(double moveSpeed) override;

        //! Returns the rotation speed
        virtual double getRotateSpeed() const override;

        //! Set the rotation speed
        virtual void setRotateSpeed(double rotateSpeed) override;

        //! Returns the zoom speed
        virtual double getZoomSpeed() const override;

        //! Set the zoom speed
        virtual void setZoomSpeed(double zoomSpeed) override;

        //! This animator will receive events when attached to the active camera
        virtual bool isEventReceiverEnabled() const override;

        //! Returns type of the scene node
        virtual irr::scene::ESCENE_NODE_ANIMATOR_TYPE getType() const override
        {
            return irr::scene::ESNAT_CAMERA_MAYA;
        }

        //! Creates a clone of this animator.
        /** Please note that you will have to drop
        (IReferenceCounted::drop()) the returned pointer after calling
        this. */
        virtual irr::scene::ISceneNodeAnimator* createClone(irr::scene::ISceneNode* node, irr::scene::ISceneManager* newManager=0) override;

        //! Enable this animator
        virtual void enableMouseControl(bool enable = true) override;

    private:

        void allKeysUp();
        bool isMouseKeyDown(irr::s32 key) const;

        bool m_mouseKeys[3];
        bool m_wheelMoving;
        irr::f32 m_wheelDirection;

        irr::gui::ICursorControl *m_cursorControl;
        irr::scene::ISceneNode *m_cameraAxis;
        irr::core::position2df m_mousePos;
        irr::core::position2df m_initialMousePosition;
        double m_zoomSpeed;
        double m_rotateSpeed;
        double m_translateSpeed;
        bool m_zooming;
        bool m_rotating;
        bool m_movingUp;
        bool m_translating;
        bool m_isEnabled;
    };

} // end namespace irr


#endif // IDYNTREE_CAMERAANIMATOR_H
