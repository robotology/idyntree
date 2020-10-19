"""
For testing iDynTree python bindings, we rely on the unittest standard python lib
"""

import sys

sys.path.append("../../python/")
sys.path.append("../../../lib/python/")
sys.path.append("../../../lib/python/Debug/")

import unittest
import numpy as np
import idyntree.swig as iDynTree
import random

class HelpersTest(unittest.TestCase):
    """Helper methods"""
    def places(self):
        return 7

    def checkApproxEqual(self, val1, val2, msg):
        self.assertAlmostEqual(val1, val2, self.places(), msg)

    def checkVectorEqual(self, v1, v2, msg):
        msgMore = "val1 = " + str(v1) + " and val2 = " + str(v2)
        for i in range(0, len(v1)):
            self.checkApproxEqual(v1[i], v2[i], msg=msg + ":" + msgMore)

    def checkMatrixEqual(self, v1, v2, msg):
        msgMore = "val1 = " + str(v1) + " and val2 = " + str(v2)
        for r in range(0, v1.rows()):
            for c in range(0, v1.cols()):
                self.checkApproxEqual(v1.getVal(r, c), v2.getVal(r, c), msg=msg + ":" + msgMore)

    """tests"""
    def testDynSizeFromPython(self):
        pos = [-1.0, 2, 3.0]
        p1 = iDynTree.VectorDynSize.FromPython(pos)
        p2 = iDynTree.VectorDynSize(3)
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

    def testDynSizePythonConstructor(self):
        pos = [-1.0, 2, 3.0]
        p1 = iDynTree.VectorDynSize(pos)
        p2 = iDynTree.VectorDynSize(3)
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

    def testFixSizeFromPython(self):
        pos = [-1.0, 2, 3.0]
        p1 = iDynTree.Vector3.FromPython(pos)
        p2 = iDynTree.Vector3()
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

    def testFixSizePythonCostructor(self):
        pos = [-1.0, 2, 3.0]
        p1 = iDynTree.Vector3(pos)
        p2 = iDynTree.Vector3()
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

    def testWrenchFromPython(self):
        pos = [-1.0, 2, 3.0, 4.0, 5.5, 6.9]
        p1 = iDynTree.Wrench.FromPython(pos)
        p2 = iDynTree.Wrench()
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        p2.setVal(3, pos[3])
        p2.setVal(4, pos[4])
        p2.setVal(5, pos[5])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

    def testWrenchPythonConstructor(self):
        pos = [-1.0, 2, 3.0, 4.0, 5.5, 6.9]
        p1 = iDynTree.Wrench(pos)
        p2 = iDynTree.Wrench()
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        p2.setVal(3, pos[3])
        p2.setVal(4, pos[4])
        p2.setVal(5, pos[5])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

    def testTwistFromPython(self):
        pos = [-1.0, 2, 3.0, 4.0, 5.5, 6.9]
        p1 = iDynTree.Twist.FromPython(pos)
        p2 = iDynTree.Twist()
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        p2.setVal(3, pos[3])
        p2.setVal(4, pos[4])
        p2.setVal(5, pos[5])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

    def testTwistPythonConstructor(self):
        pos = [-1.0, 2, 3.0, 4.0, 5.5, 6.9]
        p1 = iDynTree.Twist(pos)
        p2 = iDynTree.Twist()
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        p2.setVal(3, pos[3])
        p2.setVal(4, pos[4])
        p2.setVal(5, pos[5])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

    def testSpatialAccFromPython(self):
        pos = [-1.0, 2, 3.0, 4.0, 5.5, 6.9]
        p1 = iDynTree.SpatialAcc.FromPython(pos)
        p2 = iDynTree.SpatialAcc()
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        p2.setVal(3, pos[3])
        p2.setVal(4, pos[4])
        p2.setVal(5, pos[5])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

    def testSpatialAccPythonConstructor(self):
        pos = [-1.0, 2, 3.0, 4.0, 5.5, 6.9]
        p1 = iDynTree.SpatialAcc(pos)
        p2 = iDynTree.SpatialAcc()
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        p2.setVal(3, pos[3])
        p2.setVal(4, pos[4])
        p2.setVal(5, pos[5])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

    def testMatrixDynSizeFromPython(self):
        pos = [[-1.0, -1], [3, 5.5]]
        p1 = iDynTree.MatrixDynSize.FromPython(pos)
        p2 = iDynTree.MatrixDynSize(2, 2)
        p2.setVal(0, 0, pos[0][0])
        p2.setVal(0, 1, pos[0][1])
        p2.setVal(1, 0, pos[1][0])
        p2.setVal(1, 1, pos[1][1])
        self.checkMatrixEqual(p1, p2, "helper does not properly create matrix (matrices are not equal)")

    def testMatrixDynSizeConstructor(self):
        pos = [[-1.0, -1], [3, 5.5]]
        p1 = iDynTree.MatrixDynSize(pos)
        p2 = iDynTree.MatrixDynSize(2, 2)
        p2.setVal(0, 0, pos[0][0])
        p2.setVal(0, 1, pos[0][1])
        p2.setVal(1, 0, pos[1][0])
        p2.setVal(1, 1, pos[1][1])
        self.checkMatrixEqual(p1, p2, "helper does not properly create matrix (matrices are not equal)")

    def testMatrixFixSizeConstructor(self):
        pos = [[-1.0, -1, 6], [3, 5.5, 9.9]]
        p1 = iDynTree.Matrix2x3(pos)
        p2 = iDynTree.Matrix2x3()
        p2.setVal(0, 0, pos[0][0])
        p2.setVal(0, 1, pos[0][1])
        p2.setVal(0, 2, pos[0][2])
        p2.setVal(1, 0, pos[1][0])
        p2.setVal(1, 1, pos[1][1])
        p2.setVal(1, 2, pos[1][2])
        self.checkMatrixEqual(p1, p2, "helper does not properly create matrix (matrices are not equal)")

    def testMatrixFixSizeFromPython(self):
        pos = [[-1.0, -1, 6], [3, 5.5, 9.9]]
        p1 = iDynTree.Matrix2x3.FromPython(pos)
        p2 = iDynTree.Matrix2x3()
        p2.setVal(0, 0, pos[0][0])
        p2.setVal(0, 1, pos[0][1])
        p2.setVal(0, 2, pos[0][2])
        p2.setVal(1, 0, pos[1][0])
        p2.setVal(1, 1, pos[1][1])
        p2.setVal(1, 2, pos[1][2])
        self.checkMatrixEqual(p1, p2, "helper does not properly create matrix (matrices are not equal)")

    # Numpy test

    def testSpatialAccFromNumPy(self):
        pos = np.array([-1.0, 2, 3.0, 4.0, 5.5, 6.9])
        p1 = iDynTree.SpatialAcc.FromPython(pos)
        p2 = iDynTree.SpatialAcc()
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        p2.setVal(3, pos[3])
        p2.setVal(4, pos[4])
        p2.setVal(5, pos[5])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

    def testTwistFromNumPy(self):
        pos = np.array([-1.0, 2, 3.0, 4.0, 5.5, 6.9])
        p1 = iDynTree.Twist.FromPython(pos)
        p2 = iDynTree.Twist()
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        p2.setVal(3, pos[3])
        p2.setVal(4, pos[4])
        p2.setVal(5, pos[5])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

    def testWrenchFromNumPy(self):
        pos = np.array([-1.0, 2, 3.0, 4.0, 5.5, 6.9])
        p1 = iDynTree.Wrench.FromPython(pos)
        p2 = iDynTree.Wrench()
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        p2.setVal(3, pos[3])
        p2.setVal(4, pos[4])
        p2.setVal(5, pos[5])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

    def testFixSizeFromNumPy(self):
        pos = np.array([-1.0, 2, 3.0])
        p1 = iDynTree.Vector3.FromPython(pos)
        p2 = iDynTree.Vector3()
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

    def testDynSizeFromNumPy(self):
        pos = np.array([-1.0, 2, 3.0])
        p1 = iDynTree.VectorDynSize.FromPython(pos)
        p2 = iDynTree.VectorDynSize(3)
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

    def testMatrixDynSizeFromNumPy(self):
        pos = np.array([[-1.0, -1], [3, 5.5]])
        p1 = iDynTree.MatrixDynSize.FromPython(pos)
        p2 = iDynTree.MatrixDynSize(2, 2)
        p2.setVal(0, 0, pos[0][0])
        p2.setVal(0, 1, pos[0][1])
        p2.setVal(1, 0, pos[1][0])
        p2.setVal(1, 1, pos[1][1])
        self.checkMatrixEqual(p1, p2, "helper does not properly create matrix (matrices are not equal)")

    def testMatrixFixSizeFromNumPy(self):
        pos = np.array([[-1.0, -1, 6], [3, 5.5, 9.9]])
        p1 = iDynTree.Matrix2x3.FromPython(pos)
        p2 = iDynTree.Matrix2x3()
        p2.setVal(0, 0, pos[0][0])
        p2.setVal(0, 1, pos[0][1])
        p2.setVal(0, 2, pos[0][2])
        p2.setVal(1, 0, pos[1][0])
        p2.setVal(1, 1, pos[1][1])
        p2.setVal(1, 2, pos[1][2])
        self.checkMatrixEqual(p1, p2, "helper does not properly create matrix (matrices are not equal)")

    def testToNumPy(self):
        print("Running test testToNumPy")

        pos = np.array([1.0, 2.0, 3.0])

        p2 = iDynTree.VectorDynSize(3)
        p2[0] = pos[0]
        p2[1] = pos[1]
        p2[2] = pos[2]

        p2_asNumpy = p2.toNumPy()

        if not isinstance(p2_asNumpy, np.ndarray):
            raise TypeError("Conversion to NumPy failed")

        self.checkVectorEqual(pos, p2_asNumpy, "helper does not returned a well-formed array")


if __name__ == '__main__':
    unittest.main()
