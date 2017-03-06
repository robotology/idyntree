/*!
 * @file Transform.h
 * @author Francesco Romano
 * @copyright 2016 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2016
 *
 */

#ifndef IDYNTREE_INTERNAL_TRANSFORM_H
#define IDYNTREE_INTERNAL_TRANSFORM_H

#include <iDynTree/Core/Transform.h>

namespace internal {
    namespace kinematics {
        class Transform;
    }
}

namespace iDynTree {
    class Position;
    class Rotation;
    class Transform;
}

/*!
 * @brief Internal class to wrap am iDynTree::Transform object
 *
 * It specify the frame name to which it is related and if the transform
 * object is related to only the position, orientation or the full transform.
 */
class internal::kinematics::Transform {
public:

    /*!
     * @brief Type of the trasform
     */
    enum TransformType {
        TransformTypePosition, /*!< The trasform is related only to the position component (origin of the frame)  */
        TransformTypeRotation, /*!< The trasform is related only to the orientation component */
        TransformTypeTransform /*!< The full trasform is considered */
    };

private:
    /*! Private constructor
     *
     * @brief frameName the name of the frame
     * @brief type type of transfom
     */
    Transform(const std::string& frameName, TransformType type);

    TransformType m_type; /*!< type of transform */
    iDynTree::Transform m_transform; /*!< actual transform */
    std::string m_frameName; /*!< frame name */

public:

    /*! @brief create a position constraint for the specified frame
     *
     * @see transformConstraint
     *
     * @param frameName the name of the frame
     * @param position the position to be considered as constraint for the frame
     * @return a newly created Transform object
     */
    static Transform positionConstraint(const std::string& frameName, const iDynTree::Position &position);

    /*! @brief create an orientation constraint for the specified frame
     *
     * @see transformConstraint
     *
     * @param frameName the name of the frame
     * @param rotation the rotation to be considered as constraint for the frame
     * @return a newly created Transform object
     */
    static Transform rotationConstraint(const std::string& frameName, const iDynTree::Rotation &rotation);

    /*! @brief create a full Transform constraint for the specified frame
     *
     * Equivalent to
     * @code
     * iDynTree::Transform transform(rotation, position);
     * transformConstraint(frameName, transform.getPosition);
     * @endcode
     *
     * @param frameName the name of the frame
     * @param position the position to be considered as constraint for the frame
     * @param rotation the rotation to be considered as constraint for the frame
     * @return a newly created Transform object
     */
    static Transform transformConstraint(const std::string& frameName, const iDynTree::Position &position, const iDynTree::Rotation &rotation);

    /*! @brief create a full Transform constraint for the specified frame
     *
     * Equivalent to
     * @code
     * transformConstraint(frameName, transform.getPosition(), transform.getRotation());
     * @endcode
     * @see transformConstraint
     *
     * @param frameName the name of the frame
     * @param position the position to be considered as constraint for the frame
     * @return a newly created Transform object
     */
    static Transform transformConstraint(const std::string& frameName, const iDynTree::Transform &transform);

    /*!
     * @brief return the size of the constraint identified by this Transform
     * @note the rotation constraint is considered to be represented as a quaternion, i.e. size 4
     * @todo The above point should probably be changed if needed
     * @return size of the constraint
     */
    unsigned getSize() const;

    /*!
     * return the current type of Transform
     * @return the transform type
     */
    Transform::TransformType getType() const;

    /*! Return if the current Transform has a position constraint
     *
     * @return true if the Transform has a component in the position part. False otherwise
     */
    bool hasPositionConstraint() const;

    /*! Return if the current Transform has a rotation constraint
     *
     * @return true if the Transform has a component in the rotation part. False otherwise
     */
    bool hasRotationConstraint() const;

    /*!
     * Return the position component of the current Transform
     * @return the transform position component
     */
    const iDynTree::Position& getPosition() const;

    /*!
     * Return the rotation component of the current Transform
     * @return the transform rotation component
     */
    const iDynTree::Rotation& getRotation() const;

    /*!
     * Return the current Transform
     * @return the transform
     */
    const iDynTree::Transform& getTransform() const;

    /*!
     * Return the name of the frame associated with the current Transform
     * @return the transform frame name
     */
    const std::string& getFrameName() const;
    
};

#endif /* end of include guard: IDYNTREE_INTERNAL_TRANSFORM_H */
