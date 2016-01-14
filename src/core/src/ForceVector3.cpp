/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/ForceVector3.h>
#include <iDynTree/Core/LinearForceVector3.h>
#include <iDynTree/Core/AngularForceVector3.h>
#include <iDynTree/Core/PrivatePreProcessorUtils.h>

#define FORCEVECTOR3SEMANTICS_INSTANCE_HDR \
ForceVector3Semantics<ForceTSemantics>

#define FORCEVECTOR3_INSTANCE_HDR \
ForceVector3<ForceT>

namespace iDynTree
{
    /**
     * ForceVector3Semantics Method definitions
     */

    // constructors
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


    // semantics operations
    FORCEVECTOR3SEMANTICS_TEMPLATE_HDR
    bool FORCEVECTOR3SEMANTICS_INSTANCE_HDR::compose(const ForceVector3Semantics & op1,
                                                     const ForceVector3Semantics & op2,
                                                     ForceVector3Semantics & result)
    {
        // check semantics
        bool semantics_status =
        (   reportErrorIf(!checkEqualOrUnknown(op1.coordinateFrame,op2.coordinateFrame),
                          IDYNTREE_PRETTY_FUNCTION,
                          "multiplying two geometric relations expressed in different coordinateFrames\n")
         && reportErrorIf(!checkEqualOrUnknown(op1.body,op2.body),
                          IDYNTREE_PRETTY_FUNCTION,
                          "The bodies defined for both operands of the dot product don't match\n")
         && reportErrorIf(!checkEqualOrUnknown(op1.refBody,op2.refBody),
                          IDYNTREE_PRETTY_FUNCTION,
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
    FORCEVECTOR3_INSTANCE_HDR::ForceVector3(const double* in_data, const unsigned int in_size):
    GeomVector3<ForceT>(in_data, in_size)
    {}

    FORCEVECTOR3_TEMPLATE_HDR
    FORCEVECTOR3_INSTANCE_HDR::ForceVector3(const ForceVector3 & other):
    GeomVector3<ForceT>(other)
    {}


    /**
     * Instantiations for avoiding linker issues
     */

    template class ForceVector3Semantics<LinearForceVector3Semantics>;
    template class ForceVector3Semantics<AngularForceVector3Semantics>;

    template class ForceVector3<LinearForceVector3>;
    template class ForceVector3<AngularForceVector3>;

}