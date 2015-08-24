/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <iDynTree/Model/FixedJoint.h>
#include <iDynTree/Model/Model.h>

#include <iDynTree/Core/TestUtils.h>

#include <cstdio>
#include <cstdlib>
#include <boost/concept_check.hpp>

using namespace iDynTree;


void createCopyAndDestroy(const Model & model)
{
    Model * p_model = new Model(model);
    ASSERT_EQUAL_DOUBLE(p_model->getNrOfLinks(),model.getNrOfLinks());
    ASSERT_EQUAL_DOUBLE(p_model->getNrOfJoints(),model.getNrOfJoints());
    *p_model = model;
    ASSERT_EQUAL_DOUBLE(p_model->getNrOfLinks(),model.getNrOfLinks());
    ASSERT_EQUAL_DOUBLE(p_model->getNrOfJoints(),model.getNrOfJoints());
    delete p_model;
}

int main()
{
    double rotInertiaData[3*3] = {10.0,0.0,0.0,
                                  0.0,20.0,0.0,
                                  0.0,0.0,30.0};
    SpatialInertia inertiaLink0(1.0,Position(100,0,0),RotationalInertiaRaw(rotInertiaData,3,3));

    Link link0;
    link0.setInertia(inertiaLink0);

    Link link1(link0);

    FixedJoint fixJoint(0,1,Transform(Rotation::Identity(),Position(1,3,4)));

    {
        Model model;

        model.addLink("link0",link0);
        model.addLink("link1",link1);

        model.addJoint("fixedJoint",&fixJoint);

        ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(),2);
        ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(),1);
        ASSERT_EQUAL_DOUBLE(model.getNrOfNeighbors(0),1);
        ASSERT_EQUAL_DOUBLE(model.getNrOfNeighbors(1),1);

        createCopyAndDestroy(model);
    }

    return EXIT_SUCCESS;
}
