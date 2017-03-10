/*!
 * @file TransformConstraint.h
 * @author Francesco Romano
 * @copyright 2016 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2016
 *
 */

#ifndef IDYNTREE_INTERNAL_TRANSFORM_CONSTRAINT_H
#define IDYNTREE_INTERNAL_TRANSFORM_CONSTRAINT_H

#include <iDynTree/Core/Transform.h>

namespace internal {
    namespace kinematics {
        class TransformConstraint;
    }
}

namespace iDynTree {
    class Position;
    class Rotation;
    class Transform;
}

/*!
 * @brief Class representing a constraint (full or partial) on a transform between a given frame and the absolute frame.
 *
 *
 *
 * The constraint can be of the following type, where
 * we indicate with \f$A\f$ the absolute (world) frame and with
 * \f$C\f$ the constrained frame.
 *
 * Full Transform constraint: the transform ${}^A H_C$ is constrained to a given constant value.
 * Rotation Constraint: the rotation ${}^A R_C$ is constrained to a given constant value.
 * Position Constraint: the linear position ${}^A o_C$ is constrained to a given constant value.
 *
 */
class internal::kinematics::TransformConstraint {
public:

    /*!
     * @brief Type of the trasform constraint.
     */
    enum TransformConstraintType {
        TransformConstraintTypePosition, /*!< The trasform is related only to the position component (origin of the frame)  */
        TransformConstraintTypeRotation, /*!< The trasform is related only to the orientation component */
        TransformConstraintTypeFullTransform /*!< The full trasform is considered */
    };

private:
    /*! Private constructor
     *
     * @brief frameName the name of the frame
     * @brief type type of transfom
     */
    TransformConstraint(const std::string& frameName, TransformConstraintType type);

    TransformConstraintType m_type; /*!< type of transform */
    iDynTree::Transform m_transform; /*!< Constrained value for ${}^A H_C$. */
    std::string m_frameName; /*!< C constrained frame. */

public:

    /*! @brief create a position constraint for the specified frame
     *
     * @see fullTransformConstraint
     *
     * @param constrainedFrameName the name of the constrained frame, i.e. C
     * @param position the position to be considered as constraint for the frame
     * @return a newly created TransformConstraint object
     */
    static TransformConstraint positionConstraint(const std::string& frameName, const iDynTree::Position &position);

    /*! @brief create an orientation constraint for the specified frame
     *
     * @see fullTransformConstraint
     *
     * @param constrainedFrameName the name of the constrained frame, i.e. C
     * @param rotation the rotation to be considered as constraint for the frame
     * @return a newly created Transform object
     */
    static TransformConstraint rotationConstraint(const std::string& frameName, const iDynTree::Rotation &rotation);

    /*! @brief create a full Transform constraint for the specified frame
     *
     * Equivalent to
     * @code
     * iDynTree::Transform transform(rotation, position);
     * fullTransformConstraint(frameName, transform);
     * @endcode
     *
     * @param constrainedFrameName the name of the constrained frame, i.e. C
     * @param position the position to be considered as constraint for the frame
     * @param rotation the rotation to be considered as constraint for the frame
     * @return a newly created Transform object
     */
    static TransformConstraint fullTransformConstraint(const std::string& constrainedFrameName, const iDynTree::Position &position, const iDynTree::Rotation &rotation);

    /*! @brief create a full Transform constraint for the specified frame
     *
     * Equivalent to
     * @code
     * fullTransformConstraint(frameName, transform.getPosition(), transform.getRotation());
     * @endcode
     * @see fullTransformConstraint
     *
     * @param constrainedFrameName the name of the constrained frame, i.e. C
     * @param position the position to be considered as constraint for the frame
     * @return a newly created Transform object
     */
    static TransformConstraint fullTransformConstraint(const std::string& constrainedFrameName, const iDynTree::Transform &transform);

    /*!
     * @brief return the size of the constraint identified by this Constrained
     * @note the rotation constraint is considered to be represented as a quaternion, i.e. size 4
     * @todo The above point should probably be changed if needed
     * @return size of the constraint
     */
    unsigned getSize() const;

    /*!
     * return the current type of TransformConstraint
     * @return the transform type
     */
    TransformConstraint::TransformConstraintType getType() const;

    /*! Return if the current TransformConstraint has a position constraint
     *
     * @return true if the TransformConstraint has a component in the position part. False otherwise
     */
    bool hasPositionConstraint() const;

    /*! Return if the current TransformConstraint has a rotation constraint
     *
     * @return true if the TransformConstraint has a component in the rotation part. False otherwise
     */
    bool hasRotationConstraint() const;

    /*!
     * Return the position component of the current constrained value of the Transform.
     * @return the transform position component
     */
    const iDynTree::Position& getPosition() const;

    /*!
     * Return the rotation component of the current constrained value of the Transform.
     * @return the transform rotation component
     */
    const iDynTree::Rotation& getRotation() const;

    /*!
     * Return the current constrained value of the Transform.
     * @return the transform
     */
    const iDynTree::Transform& getTransform() const;

    /*!
     * Return the name of the constrained frame
     * @return the transform frame name
     */
    const std::string& getFrameName() const;
    
};

#endif /* end of include guard: IDYNTREE_INTERNAL_TRANSFORM_H */
