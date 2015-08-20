/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "LinearMotionVector3.h"
#include "AngularMotionVector3.h"
#include "Position.h"

namespace iDynTree
{
    /**
     * LinearMotionVector3Semantics
     */
    
    // constructors
    LinearMotionVector3Semantics::LinearMotionVector3Semantics()
    {
    }
    
    LinearMotionVector3Semantics::LinearMotionVector3Semantics(int _point, int _body, int _refBody, int _coordinateFrame):
    GeomVector3Semantics<LinearMotionVector3Semantics>(_body, _refBody, _coordinateFrame),
    point(_point)
    {
    }
    
    LinearMotionVector3Semantics::LinearMotionVector3Semantics(const LinearMotionVector3Semantics & other):
    GeomVector3Semantics<LinearMotionVector3Semantics>(other)
    {
    }
    
    LinearMotionVector3Semantics::~LinearMotionVector3Semantics()
    {
    }
    
    // Semantics operations
    bool LinearMotionVector3Semantics::changePoint(const PositionSemantics & newPoint,
                                                   const AngularMotionVector3Semantics & otherAngular,
                                                   LinearMotionVector3Semantics & resultLinear) const
    {
        return true;
    }


    /**
     * LinearMotionVector3
     */
    
    // constructors
    LinearMotionVector3::LinearMotionVector3()
    {
    }
    
    LinearMotionVector3::LinearMotionVector3(const double* in_data, const unsigned int in_size):
    MotionVector3<LinearMotionVector3, LinearMotionAssociationsT, LinearMotionVector3Semantics>(in_data, in_size)
    {
    }
    
    LinearMotionVector3::LinearMotionVector3(const LinearMotionVector3 & other):
    MotionVector3<LinearMotionVector3, LinearMotionAssociationsT, LinearMotionVector3Semantics>(other)
    {
    }
    
    LinearMotionVector3::~LinearMotionVector3()
    {
    }

    /**
     * Geometric operations
     */
    const LinearMotionVector3 LinearMotionVector3::changePoint(const Position & newPoint,
                                                               const AngularMotionVector3 & otherAngular) const
    {
        LinearMotionVector3 resultLinear;
        
        iDynTreeAssert(this->semantics.changePoint(newPoint.getSemantics(), otherAngular.getSemantics(), resultLinear.semantics));
        
        Eigen::Map<const Vector3d> newPointMap(newPoint.data());
        Eigen::Map<const Vector3d> otherAngularMap(otherAngular.data());
        Eigen::Map<const Vector3d> thisMap(this->data());
        Eigen::Map<Vector3d> resultLinearMap(resultLinear.data());
        
        resultLinearMap = thisMap + newPointMap.cross(otherAngularMap);
        
        return resultLinear;
    }
}