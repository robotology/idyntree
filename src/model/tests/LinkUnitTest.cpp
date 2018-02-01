/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Model/Link.h>

#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/TestUtils.h>

#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

int main()
{
    double rotInertiaData[3*3] = {10.0,0.0,0.0,
                                  0.0,20.0,0.0,
                                  0.0,0.0,30.0};
    SpatialInertia settedInertia(1.0,Position(100,0,0),RotationalInertiaRaw(rotInertiaData,3,3));

    Link link;

    link.setInertia(settedInertia);

    SpatialInertia gettedInertia = link.getInertia();

    ASSERT_EQUAL_MATRIX(settedInertia.asMatrix(),gettedInertia.asMatrix());

    Link copyConstructedLink(link);

    ASSERT_EQUAL_MATRIX(link.getInertia().asMatrix(),copyConstructedLink.getInertia().asMatrix());

    Link copiedLink;

    copiedLink = link;

    ASSERT_EQUAL_MATRIX(link.getInertia().asMatrix(),copiedLink.getInertia().asMatrix());

    return EXIT_SUCCESS;
}
