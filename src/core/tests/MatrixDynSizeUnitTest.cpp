/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/TestUtils.h>


using namespace iDynTree;

int main()
{
    MatrixDynSize mat;

    ASSERT_EQUAL_DOUBLE(mat.rows(),0);
    ASSERT_EQUAL_DOUBLE(mat.cols(),0);
    ASSERT_EQUAL_DOUBLE(mat.capacity(),0);

    mat.resize(10,20);

    ASSERT_EQUAL_DOUBLE(mat.rows(),10);
    ASSERT_EQUAL_DOUBLE(mat.cols(),20);
    ASSERT_EQUAL_DOUBLE(mat.capacity(),10*20);

    mat.reserve(1000);

    ASSERT_EQUAL_DOUBLE(mat.rows(),10);
    ASSERT_EQUAL_DOUBLE(mat.cols(),20);
    ASSERT_EQUAL_DOUBLE(mat.capacity(),1000);

    mat.reserve(2000);

    ASSERT_EQUAL_DOUBLE(mat.rows(),10);
    ASSERT_EQUAL_DOUBLE(mat.cols(),20);
    ASSERT_EQUAL_DOUBLE(mat.capacity(),2000);

    mat.resize(5,20);

    ASSERT_EQUAL_DOUBLE(mat.rows(),5);
    ASSERT_EQUAL_DOUBLE(mat.cols(),20);
    ASSERT_EQUAL_DOUBLE(mat.capacity(),2000);

    mat.shrink_to_fit();

    ASSERT_EQUAL_DOUBLE(mat.rows(),5);
    ASSERT_EQUAL_DOUBLE(mat.cols(),20);
    ASSERT_EQUAL_DOUBLE(mat.capacity(),100);
}
