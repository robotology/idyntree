/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/TestUtils.h>


using namespace iDynTree;

int main()
{
    VectorDynSize vec;

    ASSERT_EQUAL_DOUBLE(vec.size(),0);
    ASSERT_EQUAL_DOUBLE(vec.capacity(),0);

    vec.resize(10);

    ASSERT_EQUAL_DOUBLE(vec.size(),10);
    ASSERT_EQUAL_DOUBLE(vec.capacity(),10);

    vec.reserve(20);

    ASSERT_EQUAL_DOUBLE(vec.size(),10);
    ASSERT_EQUAL_DOUBLE(vec.capacity(),20);

    vec.reserve(30);

    ASSERT_EQUAL_DOUBLE(vec.size(),10);
    ASSERT_EQUAL_DOUBLE(vec.capacity(),30);

    vec.resize(15);

    ASSERT_EQUAL_DOUBLE(vec.size(),15);
    ASSERT_EQUAL_DOUBLE(vec.capacity(),30);

    vec.shrink_to_fit();

    ASSERT_EQUAL_DOUBLE(vec.size(),15);
    ASSERT_EQUAL_DOUBLE(vec.capacity(),15);
}
