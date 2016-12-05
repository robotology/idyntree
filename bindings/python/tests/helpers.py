'''
 For testing iDynTree python bindings, we rely on the unittest standard python lib
'''

import sys
# This test is mean to be executed from the build,
# so we add in PYTHONPATH the location of iDynTree.py and _iDynTree.so
sys.path.append("../:../../../lib/python/")

import unittest
import iDynTree; iDynTree.init_helpers()
import random

class HelpersTest(unittest.TestCase):
    '''Helper methods'''
    def places(self):
        return 7;

    def checkApproxEqual(self,val1,val2,msg):
        self.assertAlmostEqual(val1,val2,self.places(),msg);

    def checkSpatialVectorEqual(self,p1,p2,msg):
        msgMore = "val1 = " + p1.toString() + " and val2 = " + p2.toString();
        for i in range(0,3):
            self.checkApproxEqual(p1.getVal(i),p2.getVal(i),msg+":"+msgMore);

    def checkVectorEqual(self,v1,v2,msg):
        msgMore = "val1 = " + v1.toString() + " and val2 = " + v2.toString();
        for i in range(0,v1.size()):
            self.checkApproxEqual(v1.getVal(i), v2.getVal(i), msg+":"+msgMore);

    '''tests'''
    def testPyHelpers(self):
        pos = [1.0, 2.0, 3.0]
        p1 = iDynTree.VectorDynSize.fromList(pos)
        p2 = iDynTree.VectorDynSize(3)
        p2.setVal(0, pos[0])
        p2.setVal(1, pos[1])
        p2.setVal(2, pos[2])
        self.checkVectorEqual(p1, p2, "helper does not properly create vector (vectors are not equal)")

if __name__ == '__main__':
    unittest.main()
