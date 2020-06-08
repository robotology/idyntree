classdef KinDynComputations < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = KinDynComputations(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1936, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1937, self);
        self.SwigClear();
      end
    end
    function varargout = loadRobotModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1938, self, varargin{:});
    end
    function varargout = loadRobotModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1939, self, varargin{:});
    end
    function varargout = loadRobotModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1940, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1941, self, varargin{:});
    end
    function varargout = setFrameVelocityRepresentation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1942, self, varargin{:});
    end
    function varargout = getFrameVelocityRepresentation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1943, self, varargin{:});
    end
    function varargout = getNrOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1944, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreeOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1945, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1946, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1947, self, varargin{:});
    end
    function varargout = getNrOfFrames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1948, self, varargin{:});
    end
    function varargout = getFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1949, self, varargin{:});
    end
    function varargout = setFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1950, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1951, self, varargin{:});
    end
    function varargout = getRobotModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1952, self, varargin{:});
    end
    function varargout = getRelativeJacobianSparsityPattern(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1953, self, varargin{:});
    end
    function varargout = getFrameFreeFloatingJacobianSparsityPattern(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1954, self, varargin{:});
    end
    function varargout = setJointPos(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1955, self, varargin{:});
    end
    function varargout = setRobotState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1956, self, varargin{:});
    end
    function varargout = getRobotState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1957, self, varargin{:});
    end
    function varargout = getWorldBaseTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1958, self, varargin{:});
    end
    function varargout = getBaseTwist(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1959, self, varargin{:});
    end
    function varargout = getJointPos(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1960, self, varargin{:});
    end
    function varargout = getJointVel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1961, self, varargin{:});
    end
    function varargout = getModelVel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1962, self, varargin{:});
    end
    function varargout = getFrameIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1963, self, varargin{:});
    end
    function varargout = getFrameName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1964, self, varargin{:});
    end
    function varargout = getWorldTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1965, self, varargin{:});
    end
    function varargout = getWorldTransformsAsHomogeneous(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1966, self, varargin{:});
    end
    function varargout = getRelativeTransformExplicit(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1967, self, varargin{:});
    end
    function varargout = getRelativeTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1968, self, varargin{:});
    end
    function varargout = getFrameVel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1969, self, varargin{:});
    end
    function varargout = getFrameAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1970, self, varargin{:});
    end
    function varargout = getFrameFreeFloatingJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1971, self, varargin{:});
    end
    function varargout = getRelativeJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1972, self, varargin{:});
    end
    function varargout = getRelativeJacobianExplicit(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1973, self, varargin{:});
    end
    function varargout = getFrameBiasAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1974, self, varargin{:});
    end
    function varargout = getCenterOfMassPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1975, self, varargin{:});
    end
    function varargout = getCenterOfMassVelocity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1976, self, varargin{:});
    end
    function varargout = getCenterOfMassJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1977, self, varargin{:});
    end
    function varargout = getCenterOfMassBiasAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1978, self, varargin{:});
    end
    function varargout = getAverageVelocity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1979, self, varargin{:});
    end
    function varargout = getAverageVelocityJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1980, self, varargin{:});
    end
    function varargout = getCentroidalAverageVelocity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1981, self, varargin{:});
    end
    function varargout = getCentroidalAverageVelocityJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1982, self, varargin{:});
    end
    function varargout = getLinearAngularMomentum(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1983, self, varargin{:});
    end
    function varargout = getLinearAngularMomentumJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1984, self, varargin{:});
    end
    function varargout = getCentroidalTotalMomentum(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1985, self, varargin{:});
    end
    function varargout = getFreeFloatingMassMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1986, self, varargin{:});
    end
    function varargout = inverseDynamics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1987, self, varargin{:});
    end
    function varargout = generalizedBiasForces(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1988, self, varargin{:});
    end
    function varargout = generalizedGravityForces(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1989, self, varargin{:});
    end
    function varargout = generalizedExternalForces(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1990, self, varargin{:});
    end
    function varargout = inverseDynamicsInertialParametersRegressor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1991, self, varargin{:});
    end
  end
  methods(Static)
  end
end
