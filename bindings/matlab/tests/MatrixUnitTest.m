function test_suite=MatrixUnitTest
    initTestSuite

function test_sparse_matrices
    iDynTreeLoad;
    rand('state',0);
    randSparse = sprand(10,20,0.5);
    randSparseId = iDynTree.MatrixDynSize();
    randSparseId.fromMatlab(randSparse);
    assertElementsAlmostEqual(randSparseId.toMatlab(),full(randSparse))
