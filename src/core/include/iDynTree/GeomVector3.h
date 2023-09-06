// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_GEOM_VECTOR_3_H
#define IDYNTREE_GEOM_VECTOR_3_H

#include <iDynTree/VectorFixSize.h>

namespace iDynTree
{
	class Rotation;

	class GeomVector3 : public Vector3 {
	public:
		GeomVector3() = default;
		GeomVector3(const double* in_data, const unsigned int in_size);
		GeomVector3(const double x, const double y, const double z);
		GeomVector3(const Vector3 other);
        GeomVector3 changeCoordFrame(const Rotation& newCoordFrame) const;
        GeomVector3 compose(const GeomVector3& op1, const GeomVector3& op2) const;
        GeomVector3 inverse(const GeomVector3& op) const;
        double dot(const GeomVector3& other) const;
        GeomVector3 operator+(const GeomVector3& other) const;
		GeomVector3 operator-(const GeomVector3& other) const;
		GeomVector3 operator-() const;
		Rotation exp() const;
		GeomVector3 cross(const GeomVector3& other) const;

	};

	typedef GeomVector3 LinearMotionVector3;
	typedef LinearMotionVector3 LinVelocity;
	typedef LinearMotionVector3 LinAcceleration;
	typedef GeomVector3 AngularMotionVector3;
	typedef AngularMotionVector3 AngVelocity;
	typedef AngularMotionVector3 AngAcceleration;
	typedef GeomVector3 LinearForceVector3;
	typedef LinearForceVector3 LinMomentum;
	typedef LinearForceVector3 Force;
	typedef GeomVector3 AngularForceVector3;
	typedef AngularForceVector3 AngMomentum;
	typedef AngularForceVector3 Torque;
	typedef GeomVector3 MotionVector3;

}

#endif /* IDYNTREE_GEOM_VECTOR_3_H */
