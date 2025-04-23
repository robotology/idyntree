import os
import sys

# This test is meant to be executed from the build,
# so we add in sys.path the location of iDynTree.py and _iDynTree.so
# TODO: this probably does not work on Windows, fix it
sys.path.append("../../python/")
sys.path.append("../../../lib/python/")
sys.path.append("../../../lib/python/Debug/")

import unittest
import idyntree.swig as iDynTree;
import numpy as np
import random

class InvKinTest(unittest.TestCase):
    '''Helper methods'''
    def places(self):
        return 7

    def checkApproxEqual(self,val1,val2,msg):
        self.assertAlmostEqual(val1,val2,self.places(),msg)

    def checkSpatialVectorEqual(self,p1,p2,msg):
        msgMore = "val1 = " + p1.toString() + " and val2 = " + p2.toString()
        for i in range(0,3):
            self.checkApproxEqual(p1.getVal(i),p2.getVal(i),msg+":"+msgMore)

    def checkVectorEqual(self,v1,v2,msg):
        msgMore = "val1 = " + str(v1) + " and val2 = " + str(v2)
        for i in range(0,len(v1)):
            self.checkApproxEqual(v1[i], v2[i], msg+":"+msgMore)

    '''tests'''
    def testInvKin(self):
        ik = iDynTree.InverseKinematics()

        ok = ik.loadModelFromFile('./model.urdf')

        self.assertTrue(ok)

        ik.setCostTolerance(0.0001)
        ik.setConstraintsTolerance(0.00001)
        ik.setDefaultTargetResolutionMode(iDynTree.InverseKinematicsTreatTargetAsConstraintNone)
        ik.setRotationParametrization(iDynTree.InverseKinematicsRotationParametrizationRollPitchYaw)
        ik.setVerbosity(5)

        ndofs = ik.fullModel().getNrOfPosCoords()

        self.assertTrue(ndofs > 0)

        # Test joint limits
        maxPosRead = np.zeros(ndofs)
        minPosRead = np.zeros(ndofs)
        maxPosWrite = np.zeros(ndofs)
        minPosWrite = np.zeros(ndofs)
        self.assertTrue(ik.getJointLimits(minPosRead, maxPosRead))

        # Modify the vectors
        minPosWrite = minPosRead.copy()
        minPosWrite[0] = -100.0
        maxPosWrite = maxPosRead.copy()
        maxPosWrite[0] = 100.0

        # For methods that take the data in input, we can pass also a list
        self.assertTrue(ik.setJointLimits(minPosWrite, maxPosWrite))
        self.assertTrue(ik.getJointLimits(minPosRead, maxPosRead))

        self.checkVectorEqual(minPosRead, minPosWrite, "set min joint limits are not equal to the ones read")
        self.checkVectorEqual(maxPosRead, maxPosWrite, "set max joint limits are not equal to the ones read")

        print('Test of InverseKinematics completed successfully.')

if __name__ == '__main__':
    random.seed()
    unittest.main()
