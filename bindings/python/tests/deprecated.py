'''
 For testing iDynTree python bindings, we rely on the unittest standard python lib
'''

import sys

# This test is meant to be executed from the build,
# so we add in sys.path the location of iDynTree.py and _iDynTree.so
# TODO: this probably does not work on Windows, fix it
sys.path.append("../../python/");
sys.path.append("../../../lib/python/");
sys.path.append("../../../lib/python/Debug/");


import unittest
import iDynTree
import random
import numpy as np
import warnings

class DeprecationTest(unittest.TestCase):

    def checkApproxEqual(self, val1, val2, msg):
        self.assertAlmostEqual(val1, val2, self.places(), msg)

    def checkVectorEqual(self, v1, v2, msg):
        msgMore = "val1 = " + str(v1) + " and val2 = " + str(v2)
        for i in range(0, len(v1)):
            self.checkApproxEqual(v1[i], v2[i], msg=msg + ":" + msgMore)

    def testInitHelpers(self):
        print("Running test testInitHelpers")

        with warnings.catch_warnings(record=True) as w:
            print('Warning: ', w)
            # Cause all warnings to always be triggered.
            warnings.simplefilter("always")
            # Trigger a warning.
            iDynTree.init_helpers()
            # Verify some things
            assert len(w) == 1
            assert issubclass(w[-1].category, FutureWarning)

    def testFromList(self):
        print("Running test testFromList")
        iDynTree.init_helpers()
        pos = [1.0, 2.0, 3.0]
        p1 = iDynTree.VectorDynSize.fromList(pos)
        p2 = iDynTree.VectorDynSize(3)
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

    def testInitNumpyHelpers(self):
        print("Running test testInitNumpyHelpers")
        with warnings.catch_warnings(record=True) as w:
            # Cause all warnings to always be triggered.
            warnings.simplefilter("always")
            # Trigger a warning.
            iDynTree.init_numpy_helpers()
            # Verify some things
            assert len(w) == 1
            assert issubclass(w[-1].category, FutureWarning)

    def testToNumPy(self):
        print("Running test testToNumPy")
        pos = np.array([1.0, 2.0, 3.0])
        p2 = iDynTree.VectorDynSize(3)
        p2[0] = pos[0]
        p2[1] = pos[1]
        p2[2] = pos[2]

        self.checkVectorEqual(pos, p2.toNumPy(), "toNumPy does not match numpy array")

if __name__ == '__main__':
    # initalize the seed to have predictable results
    random.seed(0);
    unittest.main()
