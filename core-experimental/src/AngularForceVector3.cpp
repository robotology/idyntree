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
    GeomVector3Semantics<AngularForceVector3Semantics>(_body, _refBody, _coordinateFrame),
    point(_point)
    {
    }

    AngularForceVector3Semantics::AngularForceVector3Semantics(const AngularForceVector3Semantics & other):
    GeomVector3Semantics<AngularForceVector3Semantics>(other)
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
        return true;
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