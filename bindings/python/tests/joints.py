'''
 For testing iDynTree python bindings, we rely on the unittest standard python lib
'''

import sys

# This test is meant to be executed from the build,
# so we add in sys.path the location of iDynTree.py and _iDynTree.so
# TODO: this probably does not work on Windows, fix it
sys.path.append("../../python/")
sys.path.append("../../../lib/python/")
sys.path.append("../../../lib/python/Debug/")

import unittest
import idyntree.swig as iDynTree
import random

class JointTest(unittest.TestCase):
    '''tests'''
    def testPrismaticJoint(self):
        prismJoint = iDynTree.PrismaticJoint();

    def testRevoluteJoint(self):
        revJoint = iDynTree.RevoluteJoint();

    def testFixedJoint(self):
        fixJoint = iDynTree.FixedJoint(iDynTree.Transform.Identity());

if __name__ == '__main__':
    # initalize the seed to have predictable results
    random.seed(0);
    unittest.main()
