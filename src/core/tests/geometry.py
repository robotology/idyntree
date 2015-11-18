'''
 For testing iDynTree python bindings, we rely on the RTF (Robot Testing Framework)

 The following methods are for reporting, failure or assertions:

 RTF.setName(name)             : sets the test name (defualt is the test filename)
 RTF.testReport(msg)           : reports a informative message
 RTF.testCheck(condition, msg) : reports a failure message
 RTF.assertError(msg)          : throws an error exception with message
 RTF.asserFail(msg)            : throws a failure exception with message
'''

import iDynTree
import random

class TestCase:
    '''Helper methods'''
    def tol(self):
        return 1e-10;

    def testApproxEqual(self,val1,val2,msg):
        RTF.testCheck((abs(val1-val2) < self.tol()),msg);

    def testPointEqual(self,p1,p2,msg):
        msgMore = "val1 = " + p1.toString() + " and val2 = " + p2.toString();
        for i in range(0,3):
            RTF.testCheck((abs(p1(i)-p2(i)) < self.tol()),msg+":"+msgMore);

    def testSpatialVectorEqual(self,p1,p2,msg):
        msgMore = "val1 = " + p1.toString() + " and val2 = " + p2.toString();
        for i in range(0,3):
            RTF.testCheck((abs(p1(i)-p2(i)) < self.tol()),msg+":"+msgMore);


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
    def testInnerProductInvariance(self, nrOfTests):
        ''' Check that the inner product between a wrench and a twist
            is invariant with respect to adjoint transforms'''
        RTF.testReport("Running test testInnerProductInvariance");
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
            '''
            RTF.testReport("power1 = " + str(power1));
            RTF.testReport("power2 = " + str(power2));
            RTF.testReport("power3 = " + str(power3));
            RTF.testReport("power4 = " + str(power4));
            RTF.testReport("power5 = " + str(power5));
            RTF.testReport("power6 = " + str(power6));
            RTF.testReport("power7 = " + str(power7));
            RTF.testReport("power8 = " + str(power8));
            '''
            self.testApproxEqual(power1,power2,"testInnerProductInvariance failed");
            self.testApproxEqual(power1,power3,"testInnerProductInvariance failed");
            self.testApproxEqual(power1,power4,"testInnerProductInvariance failed");
            self.testApproxEqual(power1,power5,"testInnerProductInvariance failed");
            self.testApproxEqual(power1,power6,"testInnerProductInvariance failed");
            self.testApproxEqual(power1,power7,"testInnerProductInvariance failed");
            self.testApproxEqual(power1,power8,"testInnerProductInvariance failed");


    def testTransformInverse(self,nrOfTests):
        RTF.testReport("Running test testTransformInverse");
        for i in range(0,nrOfTests):
            #RTF.testReport("Running test testTransformInverse for position");
            p = self.randomPosition()
            T = self.randomTransform()
            pZero = iDynTree.Position()
            pZero.zero()
            self.testPointEqual(p-p,pZero,"testTransformInverse failed");
            self.testPointEqual(T.inverse()*T*p,p,"testTransformInverse failed");
            #RTF.testReport("Running test testTransformInverse for Twist");
            v = self.randomTwist()
            vZero = iDynTree.Twist()
            vZero.zero()
            self.testSpatialVectorEqual(v-v,vZero,"v-v is not zero");
            self.testSpatialVectorEqual(T.inverse()*T*v,v,"T.inverse()*T*v is not v");
            #RTF.testReport("Running test testTransformInverse for Wrench");
            f = self.randomWrench()
            fZero = iDynTree.Wrench()
            fZero.zero()
            self.testSpatialVectorEqual(f-f,fZero,"f-f is not zero");
            self.testSpatialVectorEqual(T.inverse()*T*f,f,"T.inverse()*T*f is not f");

    def run(self):
        # initalize the seed to have predictable results
        random.seed(0);
        self.testInnerProductInvariance(10);
        self.testTransformInverse(10);