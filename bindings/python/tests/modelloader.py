import sys
from pathlib import Path

# Prefer the freshly built SWIG module from the build tree.
THIS_DIR = Path(__file__).resolve().parent
SWIG_DIR = THIS_DIR.parent / "idyntree"
sys.path.insert(0, str(SWIG_DIR))

import unittest
import numpy as np
import swig as iDynTree
import random

class ModelLoaderTest(unittest.TestCase):
    """tests"""
    def testLoaderReducedModel(self):
        model_loader = iDynTree.ModelLoader()
        considered_joints_idyn = ["torso_yaw", "torso_roll"]
        model_loader.loadReducedModelFromFile("./model.urdf", considered_joints_idyn)
        self.assertEqual(len(considered_joints_idyn), model_loader.model().getNrOfJoints())


if __name__ == '__main__':
    unittest.main()
