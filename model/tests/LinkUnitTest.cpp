/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
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
