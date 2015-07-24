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

inertiaInFrame1 = iDynTree.SpatialInertia(10,iDynTree.Position(),angularInertia);

momentumInFrame1 = inertiaInFrame1*twistInFrame1;

Frame_2_1 = iDynTree.Transform();
Frame_2_1.setRotation(iDynTree.Rotation.RPY(0.6,-0.5,0.2));
Frame_2_1.setPosition(iDynTree.Position(2,4,5));

twistInFrame2 = Frame_2_1*twistInFrame1;
inertiaInFrame2 = Frame_2_1*inertiaInFrame1;
momentumInFrame2 = inertiaInFrame2*twistInFrame2;
momentumInFrame2check = Frame_2_1*momentumInFrame1;

iDynTreeAssertEqual(momentumInFrame2.getVal(0),momentumInFrame2check.getVal(0),tol,'Error in momentumInvariance, tx')
iDynTreeAssertEqual(momentumInFrame2.getVal(1),momentumInFrame2check.getVal(1),tol,'Error in momentumInvariance, ty')
iDynTreeAssertEqual(momentumInFrame2.getVal(2),momentumInFrame2check.getVal(2),tol,'Error in momentumInvariance, tz')
iDynTreeAssertEqual(momentumInFrame2.getVal(3),momentumInFrame2check.getVal(3),tol,'Error in momentumInvariance, rx')
iDynTreeAssertEqual(momentumInFrame2.getVal(4),momentumInFrame2check.getVal(4),tol,'Error in momentumInvariance, ry')
iDynTreeAssertEqual(momentumInFrame2.getVal(5),momentumInFrame2check.getVal(5),tol,'Error in momentumInvariance, rz')

disp('Test momentumInvariance completed successfully.')