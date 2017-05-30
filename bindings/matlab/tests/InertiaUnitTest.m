function test_suite=InertiaUnitTest
    initTestSuite

function test_momentum_invariance
    tol = 1e-9;
    twistInFrame1_m = [1;2;3;4;5;6];
    twistInFrame1 = iDynTree.Twist();

    twistInFrame1.setVal(0,twistInFrame1_m(1));
    twistInFrame1.setVal(1,twistInFrame1_m(2));
    twistInFrame1.setVal(2,twistInFrame1_m(3));
    twistInFrame1.setVal(3,twistInFrame1_m(4));
    twistInFrame1.setVal(4,twistInFrame1_m(5));
    twistInFrame1.setVal(5,twistInFrame1_m(6));

    angularInertia = iDynTree.RotationalInertiaRaw();
    angularInertia.setVal(0,0,1.0);
    angularInertia.setVal(1,1,2.0);
    angularInertia.setVal(2,2,3.0);
    angularInertia.setVal(0,1,0.0);
    angularInertia.setVal(0,2,0.0);
    angularInertia.setVal(1,0,0.0);
    angularInertia.setVal(1,2,0.0);
    angularInertia.setVal(2,0,0.0);
    angularInertia.setVal(2,1,0.0);

    com = iDynTree.Position();
    com.fromMatlab([1,2,3]);
    inertiaInFrame1 = iDynTree.SpatialInertia(10,com,angularInertia);

    momentumInFrame1 = inertiaInFrame1*twistInFrame1;

    Frame_2_1 = iDynTree.Transform.Identity();
    Frame_2_1.setRotation(iDynTree.Rotation.RPY(0.6,-0.5,0.2));
    Frame_2_1.setPosition(iDynTree.Position(2,4,5));

    twistInFrame2 = Frame_2_1*twistInFrame1;
    inertiaInFrame2 = Frame_2_1*inertiaInFrame1;
    momentumInFrame2 = inertiaInFrame2*twistInFrame2;
    momentumInFrame2check = Frame_2_1*momentumInFrame1;

    assertElementsAlmostEqual(momentumInFrame2.toMatlab(),momentumInFrame2check.toMatlab());

    % check also dot product invariance
    % enable this when https://github.com/robotology/idyntree/issues/105 is solved
    %energyFrame1      = twistInFrame1.dot(momentumInFrame1);
    %energyFrame1check = momentumInFrame1.dot(twistInFrame1);
    %energyFrame2      = twistInFrame2.dot(momentumInFrame2);
    %energyFrame2check = momentumInFrame2.dot(twistInFrame2);

    %iDynTreeAssertEqual(energyFrame1,energyFrame1check,tol,'Error in momentumInvariance test, dot product not invariant')
    %iDynTreeAssertEqual(energyFrame2,energyFrame2check,tol,'Error in momentumInvariance test, dot product not invariant')
    %iDynTreeAssertEqual(energyFrame1,energyFrame2,tol,'Error in momentumInvariance test, dot product not invariant')

