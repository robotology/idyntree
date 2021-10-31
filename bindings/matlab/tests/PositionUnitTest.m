function test_suite=PositionUnitTest
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite
    test_sum_of_positions

function test_sum_of_positions
    pos1_m = [10;20;30];
    pos2_m = [1;2;3];

    tol = 1e-9;

    pos1 = iDynTree.Position();
    pos2 = iDynTree.Position();

    pos1.setVal(0,pos1_m(1));
    pos1.setVal(1,pos1_m(2));
    pos1.setVal(2,pos1_m(3));

    pos2.setVal(0,pos2_m(1));
    pos2.setVal(1,pos2_m(2));
    pos2.setVal(2,pos2_m(3));

    pos3 = pos1+pos2;
    pos3_m = pos1_m+pos2_m;

    assertElementsAlmostEqual(pos3.toMatlab(),pos3_m);
