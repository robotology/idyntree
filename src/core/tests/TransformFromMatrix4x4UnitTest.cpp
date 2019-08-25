/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/SpatialMotionVector.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iostream>
#include <string>

using namespace std;
using namespace iDynTree;

void checkFromMatrix4x4()
{
    RotationRaw rot_raw(-2.449, -1.224, -2.041, 0.408, -0.707, -0.707,  0.816, 0.409, -1.154);
    PositionRaw pos_raw(1, 2, 3);
    double transform[16] = {-2.449, -1.224, -2.041, 1, 0.408, -0.707, -0.707, 2, 0.816, 0.409, -1.154, 3, 0, 0, 0, 1};

    Rotation rot = Rotation(rot_raw);
    Position pos = Position(pos_raw);

    Matrix4x4 homogeneousMatrix(transform, 4, 4);

    Transform t_posrot(rot, pos);
    Transform t_buffer(homogeneousMatrix);

    ASSERT_EQUAL_TRANSFORM(t_posrot, t_buffer);
}

void checkLogExp()
{
    PositionRaw pos_raw(0, 0, 0);
    Rotation rot = Rotation::RPY(0.0, 1.57, 0.0);
    Position pos = Position(pos_raw);
    Transform t_posrot(rot, pos);

    SpatialMotionVector v;
    v.zero();
    v.setAngularVec3(rot.log());

    Transform t_v = v.exp();

    ASSERT_EQUAL_TRANSFORM_TOL(t_posrot, t_v, 0.01);
}


int main()
{
    checkFromMatrix4x4();
    checkLogExp();
    return EXIT_SUCCESS;
}
