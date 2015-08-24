/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_FORCE_VECTOR_3_H
#define IDYNTREE_FORCE_VECTOR_3_H

#include <iDynTree/Core/GeomVector3.h>
#include <iDynTree/Core/PrivateMotionForceVertorAssociations.h>

#define FORCEVECTOR3SEMANTICS_TEMPLATE_HDR \
template <class ForceTSemantics>
#define FORCEVECTOR3SEMANTICS_INSTANCE_HDR \
ForceVector3Semantics<ForceTSemantics>

#define FORCEVECTOR3_TEMPLATE_HDR \
template <class ForceT, class ForceAssociationsT, class ForceTSemantics>
#define FORCEVECTOR3_INSTANCE_HDR \
ForceVector3<ForceT, ForceAssociationsT, ForceTSemantics>

namespace iDynTree
{
    /**
     * Template class providing the semantics for any Force relation vector.
     */
    FORCEVECTOR3SEMANTICS_TEMPLATE_HDR
    class ForceVector3Semantics: public GeomVector3Semantics<ForceTSemantics>
    {
    public:
        /**
         * Constructors:
         */
        ForceVector3Semantics();
        ForceVector3Semantics(int _body, int _refBody, int _coordinateFrame);
        ForceVector3Semantics(const ForceVector3Semantics & other);
        ~ForceVector3Semantics();
        
        /**
         * Semantics operations
         * Compute the semantics of the result given the semantics of the operands.
         */
        static bool compose(const ForceVector3Semantics & op1,
                            const ForceVector3Semantics & op2,
                            ForceVector3Semantics & result);
        
        static bool inverse(const ForceVector3Semantics & op,
                            ForceVector3Semantics & result);
    };


    /**
     * Class providing the raw coordinates and semantics for any force vector
     *
     * \ingroup iDynTreeCore
     *
     * A force vector can be used to describe a linear or angular momentum or force.
     *
     * This is a basic vector, used to implement the adjoint transformations common
     * to every force vectors.
     *
     */
    FORCEVECTOR3_TEMPLATE_HDR
    class ForceVector3: public GeomVector3<ForceT, ForceAssociationsT, ForceTSemantics>
    {
    public:
        /**
         * constructors
         */
        ForceVector3();
        ForceVector3(const double* in_data, const unsigned int in_size);
        ForceVector3(const ForceVector3 & other);
        virtual ~ForceVector3();
    };


    /**
     * ForceVector3Semantics Method definitions
     */
    
    // constructors
    FORCEVECTOR3SEMANTICS_TEMPLATE_HDR
    FORCEVECTOR3SEMANTICS_INSTANCE_HDR::ForceVector3Semantics():
    GeomVector3Semantics<ForceTSemantics>()
    {
    }
    
    FORCEVECTOR3SEMANTICS_TEMPLATE_HDR
    FORCEVECTOR3SEMANTICS_INSTANCE_HDR::ForceVector3Semantics(int _body, int _refBody, int _coordinateFrame):
    GeomVector3Semantics<ForceTSemantics>(_body, _refBody, _coordinateFrame)
    {
    }
    
    FORCEVECTOR3SEMANTICS_TEMPLATE_HDR
    FORCEVECTOR3SEMANTICS_INSTANCE_HDR::ForceVector3Semantics(const ForceVector3Semantics & other):
    GeomVector3Semantics<ForceTSemantics>(other)
    {
    }
    
    FORCEVECTOR3SEMANTICS_TEMPLATE_HDR
    FORCEVECTOR3SEMANTICS_INSTANCE_HDR::~ForceVector3Semantics()
    {
    }


    // semantics operations
    FORCEVECTOR3SEMANTICS_TEMPLATE_HDR
    bool FORCEVECTOR3SEMANTICS_INSTANCE_HDR::compose(const ForceVector3Semantics & op1,
                                                     const ForceVector3Semantics & op2,
                                                     ForceVector3Semantics & result)
    {
        // check semantics
        bool semantics_status =
        (   reportErrorIf(!checkEqualOrUnknown(op1.coordinateFrame,op2.coordinateFrame),
                          __PRETTY_FUNCTION__,
                          "multiplying two geometric relations expressed in different coordinateFrames\n")
         && reportErrorIf(!checkEqualOrUnknown(op1.body,op2.body),
                          __PRETTY_FUNCTION__,
                          "The bodies defined for both operands of the dot product don't match\n")
         && reportErrorIf(!checkEqualOrUnknown(op1.refBody,op2.refBody),
                          __PRETTY_FUNCTION__,
                          "The reference bodies defined for both operands of the dot product don't match\n"));
        
        // compute semantics
        result = op1;
        
        return semantics_status;
    }
    
    FORCEVECTOR3SEMANTICS_TEMPLATE_HDR
    bool FORCEVECTOR3SEMANTICS_INSTANCE_HDR::inverse(const ForceVector3Semantics & op,
                                                     ForceVector3Semantics & result)
    {
        // compute semantics
        result = op;
        
        return true;
    }


    /**
     * ForceVector3 Method definitions
     */

    // constructors
    FORCEVECTOR3_TEMPLATE_HDR
    FORCEVECTOR3_INSTANCE_HDR::ForceVector3(): GeomVector3<ForceT, ForceAssociationsT, ForceTSemantics>()
    {}
    
    FORCEVECTOR3_TEMPLATE_HDR
    FORCEVECTOR3_INSTANCE_HDR::ForceVector3(const double* in_data, const unsigned int in_size): GeomVector3<ForceT, ForceAssociationsT, ForceTSemantics>(in_data, in_size)
    {}
    
    FORCEVECTOR3_TEMPLATE_HDR
    FORCEVECTOR3_INSTANCE_HDR::ForceVector3(const ForceVector3 & other): GeomVector3<ForceT, ForceAssociationsT, ForceTSemantics>(other)
    {}
    
    FORCEVECTOR3_TEMPLATE_HDR
    FORCEVECTOR3_INSTANCE_HDR::~ForceVector3()
    {}
}

#endif /* IDYNTREE_FORCE_VECTOR_3_H */