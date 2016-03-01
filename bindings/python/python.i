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

%pythoncode {

#these need to be called after iDynTree module has been loaded
def init_helpers():
    def vd_fromPyList(list):
        out = VectorDynSize(len(list))
        for v in range(0,len(list)):
            out.setVal(v, list[v])
        return out
    VectorDynSize.fromPyList = staticmethod(vd_fromPyList)

    def sa_fromPyList(list):
        out = SpatialAcc()
        for v in range(0,6):
            out.setVal(v, list[v])
        return out
    SpatialAcc.fromPyList = staticmethod(sa_fromPyList)

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
}
