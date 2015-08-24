/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "AngularForceVector3.h"
#include "LinearForceVector3.h"
#include "Position.h"

namespace iDynTree
{
    /**
     * AngularForceVector3Semantics
     */
    
    // constructors
    AngularForceVector3Semantics::AngularForceVector3Semantics()
    {
    }

    AngularForceVector3Semantics::AngularForceVector3Semantics(int _point, int _body, int _refBody, int _coordinateFrame):
    ForceVector3Semantics<AngularForceVector3Semantics>(_body, _refBody, _coordinateFrame),
    point(_point)
    {
    }

    AngularForceVector3Semantics::AngularForceVector3Semantics(const AngularForceVector3Semantics & other):
    ForceVector3Semantics<AngularForceVector3Semantics>(other),
    point(other.point)
    {
    }

    AngularForceVector3Semantics::~AngularForceVector3Semantics()
    {
    }

    // Semantics operations
    bool AngularForceVector3Semantics::changePoint(const PositionSemantics & newPoint,
                                                   const LinearForceVector3Semantics & otherLinear,
                                                   AngularForceVector3Semantics & resultAngular) const
    {
        // check semantics
        bool semantics_status =
        (   reportErrorIf(!checkEqualOrUnknown(newPoint.getCoordinateFrame(),this->coordinateFrame),
                          __PRETTY_FUNCTION__,
                          "newPoint expressed in a different coordinateFrame\n")
         && reportErrorIf(!checkEqualOrUnknown(newPoint.getReferencePoint(),this->point),
                          __PRETTY_FUNCTION__,
                          "newPoint has a reference point different from the original Force vector point\n")/*
         && reportErrorIf(!checkEqualOrUnknown(newPoint.getBody(),newPoint.getRefBody()),
                          __PRETTY_FUNCTION__,
                          "newPoint point and reference point are not fixed to the same body\n")
         && reportErrorIf(!checkEqualOrUnknown(newPoint.getRefBody(),this->body),
                          __PRETTY_FUNCTION__,
                          "newPoint reference point and original Force vector point are not fixed to the same body\n")*/
         && reportErrorIf(!checkEqualOrUnknown(otherLinear.getCoordinateFrame(),this->coordinateFrame),
                          __PRETTY_FUNCTION__,
                          "otherLinear expressed in a different coordinateFrame\n")
         && reportErrorIf(!checkEqualOrUnknown(otherLinear.getBody(),this->body),
                          __PRETTY_FUNCTION__,
                          "The bodies defined for both linear and angular force vectors don't match\n")
         && reportErrorIf(!checkEqualOrUnknown(otherLinear.getRefBody(),this->refBody),
                          __PRETTY_FUNCTION__,
                          "The reference bodies defined for both linear and angular force vectors don't match\n"));
        
        // compute semantics
        resultAngular = *this;
        resultAngular.point = newPoint.getPoint();
        
        return semantics_status;
    }

    bool AngularForceVector3Semantics::compose(const AngularForceVector3Semantics & op1,
                                               const AngularForceVector3Semantics & op2,
                                               AngularForceVector3Semantics & result)
    {
        // check semantics
        bool semantics_status =
        (   reportErrorIf(!checkEqualOrUnknown(op1.point,op2.point),
                          __PRETTY_FUNCTION__,
                          "op1 point and op2 point don't match\n")
         && ForceVector3Semantics<AngularForceVector3Semantics>::compose(op1, op2, result));
        
        // compute semantics;
        result.point = op1.point;
        
        return semantics_status;
    }


    /**
     * AngularForceVector3
     */

    // constructors
    AngularForceVector3::AngularForceVector3()
    {
    }
    
    
    AngularForceVector3::AngularForceVector3(const double* in_data, const unsigned int in_size):
    ForceVector3<AngularForceVector3, AngularForceAssociationsT, AngularForceVector3Semantics>(in_data, in_size)
    {
    }
    
    AngularForceVector3::AngularForceVector3(const AngularForceVector3& other):
    ForceVector3<AngularForceVector3, AngularForceAssociationsT, AngularForceVector3Semantics>(other)
    {
    }
    
    AngularForceVector3::~AngularForceVector3()
    {
    }

    // Geometric operations
    AngularForceVector3 AngularForceVector3::changePoint(const Position & newPoint,
                                                         const LinearForceVector3 & otherLinear) const
    {
        AngularForceVector3 resultAngular;

        iDynTreeAssert(semantics.changePoint(newPoint.getSemantics(), otherLinear.getSemantics(), resultAngular.semantics));
        
        Eigen::Map<const Vector3d> newPointMap(newPoint.data());
        Eigen::Map<const Vector3d> otherLinearMap(otherLinear.data());
        Eigen::Map<const Vector3d> thisMap(this->data());
        Eigen::Map<Vector3d> resultAngularMap(resultAngular.data());
        
        resultAngularMap = thisMap + newPointMap.cross(otherLinearMap);
        
        return resultAngular;
    }

}