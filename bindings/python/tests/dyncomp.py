'''
 For testing iDynTree python bindings, we rely on the unittest standard python lib
'''

import os
# This test is mean to be executed from the build,
# so we add in PYTHONPATH the location of iDynTree.py and _iDynTree.so
os.environ["PYTHONPATH"] = os.environ["PYTHONPATH"] + ":../:../../../lib/python/"

import unittest
import idyntree.swig as iDynTree;
import numpy as np
import random

class DynCompTest(unittest.TestCase):
    '''Helper methods'''
    def checkApproxEqual(self,val1,val2,msg):
        self.assertAlmostEqual(val1,val2,self.places(),msg)

    def checkSpatialVectorEqual(self,p1,p2,msg):
        msgMore = "val1 = " + p1.toString() + " and val2 = " + p2.toString()
        for i in range(0,3):
            self.checkApproxEqual(p1.getVal(i),p2.getVal(i),msg+":"+msgMore)

    def checkVectorEqual(self,v1,v2,msg):
        msgMore = "val1 = " + v1.toString() + " and val2 = " + v2.toString()
        for i in range(0,v1.size()):
            self.checkApproxEqual(v1.getVal(i), v2.getVal(i), msg+":"+msgMore)

    '''tests'''
    def testDynComp(self):
        dynComp = iDynTree.DynamicsComputations()

        ok = dynComp.loadRobotModelFromFile('./model.urdf')

        if not(ok):
            print("Skipping testDynComp because iDynTree is compiled with IDYNTREE_USES_KDL to OFF")

        #dynComp.getFloatingBase()

        # set state
        dofs = dynComp.getNrOfDegreesOfFreedom()
        print "dofs: {}".format(dofs)
        q = iDynTree.VectorDynSize.FromPython([random.random() for i in range(0, dofs)])
        dq = iDynTree.VectorDynSize.FromPython([random.random() for i in range(0, dofs)])
        ddq = iDynTree.VectorDynSize.FromPython([random.random() for i in range(0, dofs)])

        # set gravity
        grav = iDynTree.SpatialAcc.FromPython([0.0, 0.0, -9.81, 0.0, 0.0, 0.0])
        dynComp.setRobotState(q,dq,ddq,grav)

        torques = iDynTree.VectorDynSize(dofs+6)
        baseReactionForce = iDynTree.Wrench()

        # compute id with inverse dynamics
        dynComp.inverseDynamics(torques, baseReactionForce)

        # compute id with regressors
        nrOfParams = 10*dynComp.getNrOfLinks()
        regr = iDynTree.MatrixDynSize(6+dofs,nrOfParams)
        params = iDynTree.VectorDynSize(nrOfParams)
        dynComp.getDynamicsRegressor(regr)
        dynComp.getModelDynamicsParameters(params)

        np_reg = regr.toNumPy()
        np_params = params.toNumPy()
        generalizedForces = np.dot(np_reg, np_params)

        print 'Result of inverse dynamics:'
        #print 'baseReactionForce: {} \nTorques: {}'.format(baseReactionForce,torques)
        #print 'Generalized Forces: {}'.format(generalizedForces)

        # check consistency
        self.assertTrue(
            np.allclose(np.hstack( (baseReactionForce.toNumPy(), torques.toNumPy()) ),
                       generalizedForces,
                       rtol=1e-04, atol=1e-04)
        )
        print 'Test of DynamicsComputations completed successfully.'

if __name__ == '__main__':
    random.seed()
    unittest.main()
