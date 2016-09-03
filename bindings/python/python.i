%rename(__str__) reservedToString;
%include "python/exception.i"

%{
#include <iDynTree/Core/VectorDynSize.h>
%}

namespace iDynTree
{

%extend VectorDynSize {
    double __getitem__(int i) {
        return (*self).getVal(i);
    }

    void __setitem__(int i, double val) {
        (*self).setVal(i, val);
    }
};

}

%pythoncode %{
#these need to be called after iDynTree module has been loaded
def init_helpers():
    def dyn_fromList(cls, list):
        out = cls(len(list))
        for v in range(0,len(list)):
            out.setVal(v, list[v])
        return out
    VectorDynSize.fromList = classmethod(dyn_fromList)

    def three_fromList(cls, list):
        if len(list) < 3:
            raise ValueError("list needs to have exactly 3 items!")
        out = cls()
        for v in range(0,3):
            out.setVal(v, list[v])
        return out
    LinearForceVector3.fromList = classmethod(three_fromList)
    LinearMotionVector3.fromList = classmethod(three_fromList)
    AngularForceVector3.fromList = classmethod(three_fromList)
    AngularMotionVector3.fromList = classmethod(three_fromList)

    def six_fromList(cls, list):
        if len(list) < 6:
            raise ValueError("list needs to have exactly 6 items!")
        out = cls()
        for v in range(0,6):
            out.setVal(v, list[v])
        return out
    SpatialAcc.fromList = classmethod(six_fromList)
    ClassicalAcc.fromList = classmethod(six_fromList)
    SpatialInertia.fromList = classmethod(six_fromList)
    SpatialMomentum.fromList = classmethod(six_fromList)
    SpatialMotionVector.fromList = classmethod(six_fromList)
    Twist.fromList = classmethod(six_fromList)
    Wrench.fromList = classmethod(six_fromList)

def init_numpy_helpers():
    import numpy as np

    def vecToNumPy(self):
        return np.fromstring(self.toString(), sep=' ')
    VectorDynSize.toNumPy = vecToNumPy
    Wrench.toNumPy = vecToNumPy
    Twist.toNumPy = vecToNumPy
    SpatialAcc.toNumPy = vecToNumPy
    Position.toNumPy = vecToNumPy

    def matToNumPy(self):
        return np.fromstring(self.toString(), sep=' ').reshape(self.rows(), self.cols())
    MatrixDynSize.toNumPy = matToNumPy
    Rotation.toNumPy = matToNumPy
%}
