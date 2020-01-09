/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_TRANSFORM_SEMANTICS_H
#define IDYNTREE_TRANSFORM_SEMANTICS_H

#include <string>

#include <iDynTree/Core/Utils.h>

namespace iDynTree
{
    class PositionSemantics;
    class RotationSemantics;
    class Transform;

    /**
     * Class providing the semantics for iDynTree::Transform class.
     *
     * \ingroup iDynTreeCore
     */
    class TransformSemantics
    {
    private:
        /**
         * copy assignment operator
         * We redefine this operator because the compiler is unable to generate a default
         * one, since TransformSemantics is only composed by references.
         */
        TransformSemantics & operator=(const TransformSemantics & other);

    protected:
        PositionSemantics & positionSemantics;
        RotationSemantics & rotationSemantics;

        bool check_position2rotationConsistency(const PositionSemantics& position, const RotationSemantics& rotation);

    public:
        /**
         * Constructor: initialize all semantics from a Transform object (create links)
         */
        IDYNTREE_DEPRECATED_WITH_MSG("All iDynTree semantics class and  methods will be removed in iDynTree 2.0")
        TransformSemantics(PositionSemantics & position, RotationSemantics & rotation);


        /**
         * Get the rotation part of the transform
         */
        IDYNTREE_DEPRECATED_WITH_MSG("All iDynTree semantics class and  methods will be removed in iDynTree 2.0")
        const RotationSemantics & getRotationSemantics() const;

        /**
         * Get the translation part of the transform
         */
        const PositionSemantics & getPositionSemantics() const;

        /**
         * Set the rotation part of the transform
         */
        IDYNTREE_DEPRECATED_WITH_MSG("All iDynTree semantics class and  methods will be removed in iDynTree 2.0")
        bool setRotationSemantics(const RotationSemantics & rotation);

        /**
         * Set the translation part of the transform
         */
        IDYNTREE_DEPRECATED_WITH_MSG("All iDynTree semantics class and  methods will be removed in iDynTree 2.0")
        bool setPositionSemantics(const PositionSemantics & position);

        /**
         * Semantics operations: all of them are done through the Semantics methods from
         * Position and Rotation classes, which compose the Transform class.
         */

        /**
         * overloaded operators: done through Position and Rotation classes
         */

        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}

        friend class Transform;
    };
}

#endif
