'''
 For testing iDynTree python bindings, we rely on the unittest standard python lib
'''

import sys

from pathlib import Path

# Prefer the freshly built SWIG module from the build tree.
THIS_DIR = Path(__file__).resolve().parent
SWIG_DIR = THIS_DIR.parent / "idyntree"
sys.path.insert(0, str(SWIG_DIR))

import unittest
import swig as iDynTree
import random

class JointTest(unittest.TestCase):
    '''tests'''
    def testPrismaticJoint(self):
        prismJoint = iDynTree.PrismaticJoint();

    def testRevoluteJoint(self):
        revJoint = iDynTree.RevoluteJoint();

    def testFixedJoint(self):
        fixJoint = iDynTree.FixedJoint(iDynTree.Transform.Identity());

    def testSphericalJoint(self):
        sphJoint = iDynTree.SphericalJoint();

if __name__ == '__main__':
    # initalize the seed to have predictable results
    random.seed(0);
    unittest.main()
