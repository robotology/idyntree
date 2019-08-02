function test_suite=EKFTest
    initTestSuite

function test_span
    y = iDynTree.Vector3;
    x = 2.0;
    y.setVal(1, x);
    y_span = iDynTree.DynamicSpan(y.data, y.size);
    x_from_span = y_span.getVal(1);
    assertElementsAlmostEqual(x, x_from_span);

    x1 = 20;
    y_span.setVal(0, x1);
    x1_from_span = y_span.getVal(0);
    assertElementsAlmostEqual(x1, x1_from_span);

function test_span_qekf
    ekf = iDynTree.AttitudeQuaternionEKF();
    ekf.initializeFilter();
    init_orientation = iDynTree.Vector4;
    init_orientation.setVal(0, 1.0);
    init_span = iDynTree.DynamicSpan(init_orientation.data, init_orientation.size);
    assertElementsAlmostEqual(init_span.getVal(0), 1.0);
    ekf.setInternalStateInitialOrientation(init_span);

    x = iDynTree.Vector10;
    x.zero();
    x_span = iDynTree.DynamicSpan(x.data, x.size);
    ekf.ekfGetStates(x_span);
    assertElementsAlmostEqual(x_span.getVal(0), 1.0);
