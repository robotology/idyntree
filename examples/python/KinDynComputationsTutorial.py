# -*- coding: utf-8 -*-
"""
Created on Tue Jun 23 11:35:46 2015

@author: adelpret
"""

import iDynTree
from iDynTree import KinDynComputations
from iDynTree import ModelLoader


URDF_FILE = '/home/username/path/robot.urdf';

dynComp = KinDynComputations();
mdlLoader = ModelLoader();
mdlLoader.loadModelFromFile(URDF_FILE);

dynComp.loadRobotModel(mdlLoader.model());

print "The loaded model has", dynComp.model().getNrOfDOFs(), \
    "internal degrees of freedom and",dynComp.model().getNrOfLinks(),"links."

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
ok = dynComp.getFreeFloatingJacobian("lf_foot", jac);
if( not ok ):
    print "Error in computing jacobian of frame " + "lf_foot";
else: 
    print "Jacobian of lf_foot is\n" + jac.toString();


baseAcc = iDynTree.Vector6();
baseAcc.zero();
links = dynComp.model().getNrOfLinks();
regr = iDynTree.MatrixDynSize(6+dofs,10*links);
ok = dynComp.inverseDynamicsInertialParametersRegressor(baseAcc, dds, regr);
if( not ok ):
    print "Error in computing the dynamics regressor";
else :
    print "The dynamics regressor is\n" + regr.toString();
