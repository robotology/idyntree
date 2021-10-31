function test_suite=MatrixUnitTest
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite

function test_sparse_matrices
    rand('state',0);
    randSparse = sprand(10,20,0.5);
    randSparseId = iDynTree.MatrixDynSize();
    randSparseId.fromMatlab(randSparse);
    assertElementsAlmostEqual(randSparseId.toMatlab(),full(randSparse))