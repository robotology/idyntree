import sys

sys.path.append("../../python/")
sys.path.append("../../../lib/python/")
sys.path.append("../../../lib/python/Debug/")

import os
import unittest
import idyntree.swig as iDynTree;
import numpy as np
import random

class ExtWrenchesAndJointTorquesEstimatorUnitTest(unittest.TestCase):
    def tol(self):
        return 1e-10;

    def places(self):
        return 7;

    def checkApproxEqual(self,val1,val2,msg):
        self.assertAlmostEqual(val1,val2,self.places(),msg);


    def checkVecApproxEqual(self,p1,p2,msg):
        msgMore = "val1 = " + str(p1) + " and val2 = " + str(p2)
        for i in range(0,len(p1)):
            self.checkApproxEqual(p1[i],p2[i],msg+":"+msgMore);

    def testcomputeSubModelMatrixRelatingFTSensorsMeasuresAndKinematics(self):

        # Define considered joints
        consideredJoints = iDynTree.StringVector()
        consideredJoints.push_back("torso_pitch")
        consideredJoints.push_back("torso_roll")
        consideredJoints.push_back("torso_yaw")
        consideredJoints.push_back("neck_pitch")
        consideredJoints.push_back("neck_roll")
        consideredJoints.push_back("neck_yaw")
        consideredJoints.push_back("l_shoulder_pitch")
        consideredJoints.push_back("l_shoulder_roll")
        consideredJoints.push_back("l_shoulder_yaw")
        consideredJoints.push_back("l_elbow")
        consideredJoints.push_back("r_shoulder_pitch")
        consideredJoints.push_back("r_shoulder_roll")
        consideredJoints.push_back("r_shoulder_yaw")
        consideredJoints.push_back("r_elbow")
        consideredJoints.push_back("l_hip_pitch")
        consideredJoints.push_back("l_hip_roll")
        consideredJoints.push_back("l_hip_yaw")
        consideredJoints.push_back("l_knee")
        consideredJoints.push_back("l_ankle_pitch")
        consideredJoints.push_back("l_ankle_roll")
        consideredJoints.push_back("r_hip_pitch")
        consideredJoints.push_back("r_hip_roll")
        consideredJoints.push_back("r_hip_yaw")
        consideredJoints.push_back("r_knee")
        consideredJoints.push_back("r_ankle_pitch")
        consideredJoints.push_back("r_ankle_roll")

        # Load model
        estimator = iDynTree.ExtWrenchesAndJointTorquesEstimator()
        testModel = os.path.join(os.path.dirname(__file__), "../../../../src/tests/data/iCubDarmstadt01.urdf")
        ok = estimator.loadModelAndSensorsFromFileWithSpecifiedDOFs(testModel, consideredJoints)
        self.assertTrue(ok)

        # Set random state
        dofs = estimator.model().getNrOfDOFs()
        q = iDynTree.JointPosDoubleArray.FromPython([random.random() for i in range(0, dofs)])
        dq = iDynTree.JointDOFsDoubleArray.FromPython([random.random() for i in range(0, dofs)])
        ddq = iDynTree.JointDOFsDoubleArray.FromPython([random.random() for i in range(0, dofs)])
        grav = iDynTree.Vector3.FromPython([0.0, 0.0, -9.81])

        root_link_index = estimator.model().getFrameIndex("root_link")
        estimator.updateKinematicsFromFixedBase(q,dq,ddq,root_link_index,grav)

        # Set unknown contact forces
        fullBodyUnknowns = iDynTree.LinkUnknownWrenchContacts(estimator.model())
        unknown = iDynTree.UnknownWrenchContact()
        unknown.unknownType = iDynTree.FULL_WRENCH
        unknown.contactPoint.zero()
        fullBodyUnknowns.addNewContactInFrame(estimator.model(), root_link_index, unknown)

        # Compute expected FT measures
        # First of all, we build the 6*nrOfFTsensors vector composed by the known FT sensors measures
        sensOffset = iDynTree.SensorsMeasurements(estimator.model().sensors())
        estimatedContactWrenches = iDynTree.LinkContactWrenches(estimator.model())
        estimatedJointTorques = iDynTree.JointDOFsDoubleArray(estimator.model())

        ok = estimator.computeExpectedFTSensorsMeasurements(fullBodyUnknowns, sensOffset, estimatedContactWrenches, estimatedJointTorques)
        self.assertTrue(ok)

        nrOfFTSensors = estimator.model().sensors().getNrOfSensors(iDynTree.SIX_AXIS_FORCE_TORQUE)

        w = np.zeros(6*nrOfFTSensors)
        for ft in range(0, nrOfFTSensors):
            ft_meas = iDynTree.Wrench()
            sensOffset.getMeasurement(iDynTree.SIX_AXIS_FORCE_TORQUE, ft, ft_meas)
            w[range(6*ft, 6*ft+6)] = ft_meas.toNumPy()

        A = iDynTree.MatrixDynSizeVector()
        b = iDynTree.VectorDynSizeVector()
        subModelIDs = iDynTree.IndexVector()
        baseLinkIndeces = iDynTree.IndexVector()


        # Test computeSubModelMatrixRelatingFTSensorsMeasuresAndKinematics
        ok = estimator.computeSubModelMatrixRelatingFTSensorsMeasuresAndKinematics(fullBodyUnknowns, A, b, subModelIDs, baseLinkIndeces)
        self.assertTrue(ok)
        self.assertEqual(A.size(), b.size())
        self.assertEqual(A.size(), subModelIDs.size())
        self.assertEqual(A.size(), baseLinkIndeces.size())

        for i in range(0, A.size()):
            bNumPy = b[i].toNumPy()
            bCheck = A[i].toNumPy()@w
            self.checkVecApproxEqual(bNumPy, bCheck, "computeSubModelMatrixRelatingFTSensorsMeasuresAndKinematics check failed")


if __name__ == '__main__':
    # initalize the seed to have predictable results
    random.seed(0)
    unittest.main()
