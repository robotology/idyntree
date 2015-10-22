# -*- coding: utf-8 -*-
"""
Created on Tue Jun 23 11:35:46 2015

@author: adelpret
"""

import iDynTree
from iDynTree import DynamicsComputations

URDF_FILE = '/home/username/path/robot.urdf';

dynComp = DynamicsComputations();
dynComp.loadRobotModelFromFile(URDF_FILE);
print "The loaded model has", dynComp.getNrOfDegreesOfFreedom(), \
    "internal degrees of freedom and",dynComp.getNrOfLinks(),"links."

dofs = dynComp.getNrOfDegreesOfFreedom();
q = iDynTree.VectorDynSize(dofs);
dq = iDynTree.VectorDynSize(dofs);
ddq = iDynTree.VectorDynSize(dofs);
for dof in range(dofs):
    # For the sake of the example, we fill the joints vector with gibberish data (remember in any case
    # that all quantities are expressed in radians-based units 
    q.setVal(dof, 1.0);
    dq.setVal(dof, 0.4);
    ddq.setVal(dof, 0.3);


# The spatial acceleration is a 6d acceleration vector. 
# For all 6d quantities, we use the linear-angular serialization
# (the first three value are for the linear quantity, the 
#  the last  three values are for the angular quantity)
gravity = iDynTree.SpatialAcc();
gravity.setVal(2, -9.81);
dynComp.setRobotState(q,dq,ddq,gravity);

jac = iDynTree.MatrixDynSize(6,6+dofs);
ok = dynComp.getFrameJacobian("lf_foot", jac);
if( not ok ):
    print "Error in computing jacobian of frame " + "lf_foot";
else: 
    print "Jacobian of lf_foot is\n" + jac.toString();
    
links = dynComp.getNrOfLinks();
regr = iDynTree.MatrixDynSize(6+dofs,10*links);
ok = dynComp.getDynamicsRegressor(regr);
if( not ok ):
    print "Error in computing the dynamics regressor";
else :
    print "The dynamics regressor is\n" + regr.toString();