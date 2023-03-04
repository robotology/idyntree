import sys

sys.path.append("../../python/")
sys.path.append("../../../lib/python/")
sys.path.append("../../../lib/python/Debug/")

import unittest
import numpy as np
import idyntree.swig as iDynTree
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
