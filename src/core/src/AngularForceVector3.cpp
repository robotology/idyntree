/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/AngularForceVector3.h>
#include <iDynTree/Core/LinearForceVector3.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/PrivatePreProcessorUtils.h>
#include <iDynTree/Core/PrivateSemanticsMacros.h>

#include <Eigen/Dense>

namespace iDynTree
{
    typedef Eigen::Matrix<double,3,1> Vector3d;

    /**
     * AngularForceVector3Semantics
     */

    // constructors
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

    // Semantics operations
    bool AngularForceVector3Semantics::changePoint(const PositionSemantics & newPoint,
                                                   const LinearForceVector3Semantics & otherLinear,
                                                   AngularForceVector3Semantics & resultAngular) const
    {
        // check semantics
        bool semantics_status =
        (   reportErrorIf(!checkEqualOrUnknown(newPoint.getCoordinateFrame(),this->coordinateFrame),
                          IDYNTREE_PRETTY_FUNCTION,
                          "newPoint expressed in a different coordinateFrame\n")
         && reportErrorIf(!checkEqualOrUnknown(newPoint.getReferencePoint(),this->point),
                          IDYNTREE_PRETTY_FUNCTION,
                          "newPoint has a reference point different from the original Force vector point\n")/*
         && reportErrorIf(!checkEqualOrUnknown(newPoint.getBody(),newPoint.getRefBody()),
                          IDYNTREE_PRETTY_FUNCTION,
                          "newPoint point and reference point are not fixed to the same body\n")
         && reportErrorIf(!checkEqualOrUnknown(newPoint.getRefBody(),this->body),
                          IDYNTREE_PRETTY_FUNCTION,
                          "newPoint reference point and original Force vector point are not fixed to the same body\n")*/
         && reportErrorIf(!checkEqualOrUnknown(otherLinear.getCoordinateFrame(),this->coordinateFrame),
                          IDYNTREE_PRETTY_FUNCTION,
                          "otherLinear expressed in a different coordinateFrame\n")
         && reportErrorIf(!checkEqualOrUnknown(otherLinear.getBody(),this->body),
                          IDYNTREE_PRETTY_FUNCTION,
                          "The bodies defined for both linear and angular force vectors don't match\n")
         && reportErrorIf(!checkEqualOrUnknown(otherLinear.getRefBody(),this->refBody),
                          IDYNTREE_PRETTY_FUNCTION,
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
                          IDYNTREE_PRETTY_FUNCTION,
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
    AngularForceVector3::AngularForceVector3(const double x, const double y, const double z)
    {
        this->m_data[0] = x;
        this->m_data[1] = y;
        this->m_data[2] = z;
    }

    AngularForceVector3::AngularForceVector3(const double* in_data, const unsigned int in_size):
    ForceVector3<AngularForceVector3>(in_data, in_size)
    {
    }

    AngularForceVector3::AngularForceVector3(const AngularForceVector3& other):
    ForceVector3<AngularForceVector3>(other)
    {
    }

    AngularForceVector3::AngularForceVector3(const Vector3& other):
    ForceVector3<AngularForceVector3>(other.data(), other.size())
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
