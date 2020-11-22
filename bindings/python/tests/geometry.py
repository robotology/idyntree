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

class PositionTest(unittest.TestCase):
    '''Helper methods'''
    def tol(self):
        return 1e-10;

    def places(self):
        return 7;

    def checkApproxEqual(self,val1,val2,msg):
        self.assertAlmostEqual(val1,val2,self.places(),msg);

    def checkPointEqual(self,p1,p2,msg):
        msgMore = "val1 = " + p1.toString() + " and val2 = " + p2.toString();
        for i in range(0,3):
            self.checkApproxEqual(p1.getVal(i),p2.getVal(i),msg+":"+msgMore);

    def checkSpatialVectorEqual(self,p1,p2,msg):
        msgMore = "val1 = " + p1.toString() + " and val2 = " + p2.toString();
        for i in range(0,3):
            self.checkApproxEqual(p1.getVal(i),p2.getVal(i),msg+":"+msgMore);

    def randomTransform(self):
        ''' Return a random Transform '''
        res = iDynTree.Transform();
        res.setPosition(iDynTree.Position(random.uniform(-10,10),random.uniform(-10,10),random.uniform(-10,10)));
        res.setRotation(iDynTree.Rotation.RPY(random.uniform(-10,10),random.uniform(-10,10),random.uniform(-10,10)));
        return res;

    def randomTwist(self):
        ''' Return a random Twist '''
        res = iDynTree.Twist()
        for i in range(0,6):
            res.setVal(i,random.uniform(-10,10));
        return res;

    def randomWrench(self):
        ''' Return a random wrench '''
        res = iDynTree.Wrench()
        for i in range(0,6):
            res.setVal(i,random.uniform(-10,10));
        return res;

    def randomPosition(self):
        '''Return a random position vector'''
        res = iDynTree.Position()
        for i in range(0,3):
            res.setVal(i,random.uniform(-10,10));

        return res

    '''tests'''

    def testInnerProductInvariance(self, nrOfTests=5):
        ''' Check that the inner product between a wrench and a twist
        is invariant with respect to adjoint transforms'''
        print("Running test testInnerProductInvariance");
        for i in range(0,nrOfTests):
            f = self.randomWrench();
            v = self.randomTwist();
            T = self.randomTransform();
            power1 = f.dot(v);
            power2 = v.dot(f);
            power3 = (T*f).dot(T*v);
            power4 = (T*v).dot(T*f);
            T_id = T*T.inverse();
            power5 = (T_id*f).dot(T_id*v);
            power6 = (T_id*v).dot(T_id*f);
            power7 = (T.inverse()*f).dot(T.inverse()*v);
            power8 = (T.inverse()*v).dot(T.inverse()*f)
            self.checkApproxEqual(power1,power2,"testInnerProductInvariance failed");
            self.checkApproxEqual(power1,power3,"testInnerProductInvariance failed");
            self.checkApproxEqual(power1,power4,"testInnerProductInvariance failed");
            self.checkApproxEqual(power1,power5,"testInnerProductInvariance failed");
            self.checkApproxEqual(power1,power6,"testInnerProductInvariance failed");
            self.checkApproxEqual(power1,power7,"testInnerProductInvariance failed");
            self.checkApproxEqual(power1,power8,"testInnerProductInvariance failed");


    def testTransformInverse(self,nrOfTests=5):
        print("Running test testTransformInverse");
        for i in range(0,nrOfTests):
            #RTF.testReport("Running test testTransformInverse for position");
            p = self.randomPosition()
            T = self.randomTransform()
            pZero = iDynTree.Position()
            pZero.zero()
            self.checkPointEqual(p-p,pZero,"testTransformInverse failed");
            self.checkPointEqual(T.inverse()*T*p,p,"testTransformInverse failed");
            #RTF.testReport("Running test testTransformInverse for Twist");
            v = self.randomTwist()
            vZero = iDynTree.Twist()
            vZero.zero()
            self.checkSpatialVectorEqual(v-v,vZero,"v-v is not zero");
            self.checkSpatialVectorEqual(T.inverse()*T*v,v,"T.inverse()*T*v is not v");
            #RTF.testReport("Running test testTransformInverse for Wrench");
            f = self.randomWrench()
            fZero = iDynTree.Wrench()
            fZero.zero()
            self.checkSpatialVectorEqual(f-f,fZero,"f-f is not zero");
            self.checkSpatialVectorEqual(T.inverse()*T*f,f,"T.inverse()*T*f is not f");

if __name__ == '__main__':
    # initalize the seed to have predictable results
    random.seed(0);
    unittest.main()
