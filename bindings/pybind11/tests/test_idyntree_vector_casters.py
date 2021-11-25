"""Tests for idyntree-vector-casters."""
import unittest

from idyntree_pybind11_test.vector_casters import TestClass
import numpy as np


class TestClassTest(unittest.TestCase):

  def test_vector_fix(self):
    obj = TestClass()
    vector = [1., 2., 3.]
    obj.vector_fix = vector
    self.assertEqual(len(obj.vector_fix), 3)
    self.assertEqual(list(obj.vector_fix), list(vector))

  def test_vector_dyn(self):
    obj = TestClass()
    vector = [1., 2., 3., 5., 6., 7.]
    obj.vector_dyn = vector
    self.assertEqual(len(obj.vector_dyn), len(vector))
    self.assertEqual(list(obj.vector_dyn), list(vector))

  def test_matrix_fix(self):
    obj = TestClass()
    matrix = np.random.rand(4, 5)
    obj.matrix_fix = matrix
    self.assertEqual(obj.matrix_fix.shape, matrix.shape)
    for i in range(4):
      for j in range(3):
        self.assertEqual(obj.matrix_fix[i,j], matrix[i,j])

  def test_matrix_dyn(self):
    obj = TestClass()
    matrix = np.random.rand(10, 10)
    obj.matrix_dyn = matrix
    self.assertEqual(obj.matrix_dyn.shape, matrix.shape)
    for i in range(10):
      for j in range(10):
        self.assertEqual(obj.matrix_dyn[i,j], matrix[i,j])

if __name__ == "__main__":
  unittest.main()

