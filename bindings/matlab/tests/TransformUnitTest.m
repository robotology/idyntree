function test_suite=TransformUnitTest
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite

function test_pos_twist_wrench_invariance
    posInFrame1_m = [1;2;3;1];

    posInFrame1 = iDynTree.Position();
    posInFrame1.setVal(0,posInFrame1_m(1));
    posInFrame1.setVal(1,posInFrame1_m(2));
    posInFrame1.setVal(2,posInFrame1_m(3));

    twistInFrame1_m = [1;2;3;4;5;6];
    twistInFrame1 = iDynTree.Twist();

    twistInFrame1.setVal(0,twistInFrame1_m(1));
    twistInFrame1.setVal(1,twistInFrame1_m(2));
    twistInFrame1.setVal(2,twistInFrame1_m(3));
    twistInFrame1.setVal(3,twistInFrame1_m(4));
    twistInFrame1.setVal(4,twistInFrame1_m(5));
    twistInFrame1.setVal(5,twistInFrame1_m(6));

    wrenchInFrame1_m = [1;2;3;4;5;6];
    wrenchInFrame1 = iDynTree.Wrench();

    wrenchInFrame1.setVal(0,wrenchInFrame1_m(1));
    wrenchInFrame1.setVal(1,wrenchInFrame1_m(2));
    wrenchInFrame1.setVal(2,wrenchInFrame1_m(3));
    wrenchInFrame1.setVal(3,wrenchInFrame1_m(4));
    wrenchInFrame1.setVal(4,wrenchInFrame1_m(5));
    wrenchInFrame1.setVal(5,wrenchInFrame1_m(6));

    Frame_2_1 = iDynTree.Transform();
    Frame_2_1.setRotation(iDynTree.Rotation.RPY(0.6,-0.5,0.2));
    Frame_2_1.setPosition(iDynTree.Position(2,4,5));

    posInFrame2 = (Frame_2_1*posInFrame1);
    posInFrame2_m = posInFrame2.toMatlab();
    posInFrame2_m(4) = 1;

    Frame_2_1Hom = Frame_2_1.asHomogeneousTransform();
    posInFrame2_matlab = (Frame_2_1Hom.toMatlab())*(posInFrame1_m);

    assertElementsAlmostEqual(posInFrame2_m,posInFrame2_matlab);

    twistInFrame2 = (Frame_2_1*twistInFrame1);
    twistInFrame2_m = twistInFrame2.asVector().toMatlab();

    Frame_2_1Adj = Frame_2_1.asAdjointTransform();
    twistInFrame2_matlab = (Frame_2_1Adj.toMatlab())*(twistInFrame1.asVector().toMatlab());

    assertElementsAlmostEqual(twistInFrame2_m,twistInFrame2_matlab);

    wrenchInFrame2 = (Frame_2_1*wrenchInFrame1);
    wrenchInFrame2_m = wrenchInFrame2.asVector().toMatlab();

    Frame21AdjWrench = Frame_2_1.asAdjointTransformWrench();
    wrenchInFrame2_matlab = (Frame21AdjWrench.toMatlab())*(wrenchInFrame1.asVector().toMatlab());

    assertElementsAlmostEqual(wrenchInFrame2_m,wrenchInFrame2_matlab);