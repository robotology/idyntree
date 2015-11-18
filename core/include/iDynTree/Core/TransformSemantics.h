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