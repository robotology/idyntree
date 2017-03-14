from sympy import *

def skewMatrix(vec):
    wS = zeros(3);
    wS[0,1] = - vec[2];
    wS[0,2] = vec[1];
    wS[1,2] = - vec[0];
    wS[1,0] = vec[2]; 
    wS[2,0] = - vec[1];
    wS[2,1] = vec[0];
    return wS;

def diagMatrix(vec):
    wS = zeros(3)
    wS[0,0] = vec[0]
    wS[1,1] = vec[1] 
    wS[2,2] = vec[2]
    return wS 

def unskew(mat):
    wUnSkewed    = zeros(3,1)
    wUnSkewed[0] = (mat[2,1]-mat[1,2])/2;
    wUnSkewed[1] = (mat[0,2]-mat[2,0])/2;
    wUnSkewed[2] = (mat[1,0]-mat[0,1])/2;
    return wUnSkewed;
    
def symbVec(vec):
    vecS = zeros(3,1)
    vecS[0] = vec[0];
    vecS[1] = vec[1];
    vecS[2] = vec[2];
    return vecS;

def RotAxis(direction,angle):
    R = eye(3) + skewMatrix(direction)*sin(angle) + skewMatrix(direction)*skewMatrix(direction)*(1-cos(angle));
    return R;

def RPYtoRot(rpy):
    xAxis = symbVec([1,0,0]);
    yAxis = symbVec([0,1,0]);
    zAxis = symbVec([0,0,1]);
    return RotAxis(zAxis,rpy[2])*RotAxis(yAxis,rpy[1])*RotAxis(xAxis,rpy[0])


r, p, y = symbols('r p y')
rpy = symbVec([r,p,y]);
Rrpy = RPYtoRot(rpy);

RPYRightTrivializedDerivative = zeros(3,3)
RPYRightTrivializedDerivative[:,0] = simplify(unskew(diff(Rrpy,r)*Rrpy.transpose()));
RPYRightTrivializedDerivative[:,1] = simplify(unskew(diff(Rrpy,p)*Rrpy.transpose()));
RPYRightTrivializedDerivative[:,2] = simplify(unskew(diff(Rrpy,y)*Rrpy.transpose()));

RPYRightTrivializedDerivative
RPYRightTrivializedDerivativeInverse = simplify(RPYRightTrivializedDerivative.inv())
