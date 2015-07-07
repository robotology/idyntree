/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_TRANSFORM_SEMANTICS_H
#define IDYNTREE_TRANSFORM_SEMANTICS_H

#include <string>

namespace iDynTree
{
    class PositionSemantics;
    class RotationSemantics;

    /**
     * Class providing the semantics for iDynTree::Transform class.
     *
     * \ingroup iDynTreeCore
     */
    class TransformSemantics
    {
    protected:
        PositionSemantics & positionSemantics;
        RotationSemantics & rotationSemantics;

        bool check_position2rotationConsistency(const PositionSemantics& position, const RotationSemantics& rotation);
        
    public:
        /**
         * Default constructor: initialize all semantics from a Transform object (create links)
         */
        TransformSemantics(PositionSemantics & position, RotationSemantics & rotation);

        /**
         * Destructor
         */
        virtual ~TransformSemantics();


        /**
         * Get the rotation part of the transform
         */
        const RotationSemantics & getRotationSemantics() const;

        /**
         * Get the translation part of the transform
         */
        const PositionSemantics & getPositionSemantics() const;

        /**
         * Set the rotation part of the transform
         */
        bool setRotationSemantics(const RotationSemantics & rotation);

        /**
         * Set the translation part of the transform
         */
        bool setPositionSemantics(const PositionSemantics & position);

        /**
         * Semantics operations: all of them are done through the Semantics methods from
         * Position and Rotation classes, which compose the Transform class.
         */

        /**
         * overloaded operators: done through Position and Rotation classes
         */

        /**
         * copy assignment operator
         */
        TransformSemantics & operator= (const TransformSemantics & semantics);
        
        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}
    };
}

#endif