# -*- coding: utf-8 -*-
"""
To run this example, first install iDynTree for python,
then navigate to this directory and run:
python KinDynComputationsTutorial.py

Otherwise, modify the URDF_FILE parameter to point to an existing
URDF in your system.
"""

import idyntree.bindings as iDynTree


URDF_FILE = '../../src/tests/data/icub.urdf ';

dynComp = iDynTree.KinDynComputations();
mdlLoader = iDynTree.ModelLoader();
mdlLoader.loadModelFromFile(URDF_FILE);

dynComp.loadRobotModel(mdlLoader.model());

print("The loaded model has", dynComp.model().getNrOfDOFs(), \
    "internal degrees of freedom and",dynComp.model().getNrOfLinks(),"links.")

dofs = dynComp.model().getNrOfDOFs();
s = iDynTree.VectorDynSize(dofs);
ds = iDynTree.VectorDynSize(dofs);
dds = iDynTree.VectorDynSize(dofs);
for dof in range(dofs):
    # For the sake of the example, we fill the joints vector with gibberish data (remember in any case
    # that all quantities are expressed in radians-based units
    s.setVal(dof, 1.0);
    ds.setVal(dof, 0.4);
    dds.setVal(dof, 0.3);


# The gravity acceleration is a 3d acceleration vector.
gravity = iDynTree.Vector3();
gravity.zero();
gravity.setVal(2, -9.81);
dynComp.setRobotState(s,ds,gravity);

jac = iDynTree.MatrixDynSize(6,6+dofs);
ok = dynComp.getFrameFreeFloatingJacobian("l_sole", jac);
if( not ok ):
    print("Error in computing jacobian of frame " + "l_sole");
else:
    print("Jacobian of lf_foot is\n" + jac.toString());


root_H_sole = dynComp.getRelativeTransform("root_link", "l_sole");
root_R_sole = root_H_sole.getRotation()
if( not ok ):
    print("Error in computing jacobian of frame " + "l_sole");
else:
    print("Orientation between root_link and l_sole is\n" + str(root_R_sole.toNumPy()));
    print("In RPY format:\n" + str(root_R_sole.asRPY().toNumPy()))

baseAcc = iDynTree.Vector6();
baseAcc.zero();
links = dynComp.model().getNrOfLinks();
regr = iDynTree.MatrixDynSize(6+dofs,10*links);
ok = dynComp.inverseDynamicsInertialParametersRegressor(baseAcc, dds, regr);
# The Dynamics Regressor is quite big, uncomment if you really want to print it
#if( not ok ):
#    print("Error in computing the dynamics regressor");
#else :
#    print("The dynamics regressor is\n" + regr.toString());
