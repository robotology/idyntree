classdef InverseKinematics < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = InverseKinematics(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(2040, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(2041, self);
        self.SwigClear();
      end
    end
    function varargout = loadModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2042, self, varargin{:});
    end
    function varargout = setModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2043, self, varargin{:});
    end
    function varargout = setJointLimits(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2044, self, varargin{:});
    end
    function varargout = getJointLimits(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2045, self, varargin{:});
    end
    function varargout = clearProblem(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2046, self, varargin{:});
    end
    function varargout = setFloatingBaseOnFrameNamed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2047, self, varargin{:});
    end
    function varargout = setCurrentRobotConfiguration(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2048, self, varargin{:});
    end
    function varargout = setJointConfiguration(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2049, self, varargin{:});
    end
    function varargout = setRotationParametrization(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2050, self, varargin{:});
    end
    function varargout = rotationParametrization(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2051, self, varargin{:});
    end
    function varargout = setMaxIterations(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2052, self, varargin{:});
    end
    function varargout = maxIterations(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2053, self, varargin{:});
    end
    function varargout = setMaxCPUTime(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2054, self, varargin{:});
    end
    function varargout = maxCPUTime(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2055, self, varargin{:});
    end
    function varargout = setCostTolerance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2056, self, varargin{:});
    end
    function varargout = costTolerance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2057, self, varargin{:});
    end
    function varargout = setConstraintsTolerance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2058, self, varargin{:});
    end
    function varargout = constraintsTolerance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2059, self, varargin{:});
    end
    function varargout = setVerbosity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2060, self, varargin{:});
    end
    function varargout = linearSolverName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2061, self, varargin{:});
    end
    function varargout = setLinearSolverName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2062, self, varargin{:});
    end
    function varargout = addFrameConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2063, self, varargin{:});
    end
    function varargout = addFramePositionConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2064, self, varargin{:});
    end
    function varargout = addFrameRotationConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2065, self, varargin{:});
    end
    function varargout = activateFrameConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2066, self, varargin{:});
    end
    function varargout = deactivateFrameConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2067, self, varargin{:});
    end
    function varargout = isFrameConstraintActive(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2068, self, varargin{:});
    end
    function varargout = addCenterOfMassProjectionConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2069, self, varargin{:});
    end
    function varargout = getCenterOfMassProjectionMargin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2070, self, varargin{:});
    end
    function varargout = getCenterOfMassProjectConstraintConvexHull(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2071, self, varargin{:});
    end
    function varargout = addTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2072, self, varargin{:});
    end
    function varargout = addPositionTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2073, self, varargin{:});
    end
    function varargout = addRotationTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2074, self, varargin{:});
    end
    function varargout = updateTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2075, self, varargin{:});
    end
    function varargout = updatePositionTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2076, self, varargin{:});
    end
    function varargout = updateRotationTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2077, self, varargin{:});
    end
    function varargout = setDefaultTargetResolutionMode(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2078, self, varargin{:});
    end
    function varargout = defaultTargetResolutionMode(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2079, self, varargin{:});
    end
    function varargout = setTargetResolutionMode(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2080, self, varargin{:});
    end
    function varargout = targetResolutionMode(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2081, self, varargin{:});
    end
    function varargout = setDesiredFullJointsConfiguration(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2082, self, varargin{:});
    end
    function varargout = setDesiredReducedJointConfiguration(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2083, self, varargin{:});
    end
    function varargout = setFullJointsInitialCondition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2084, self, varargin{:});
    end
    function varargout = setReducedInitialCondition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2085, self, varargin{:});
    end
    function varargout = solve(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2086, self, varargin{:});
    end
    function varargout = getFullJointsSolution(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2087, self, varargin{:});
    end
    function varargout = getReducedSolution(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2088, self, varargin{:});
    end
    function varargout = getPoseForFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2089, self, varargin{:});
    end
    function varargout = fullModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2090, self, varargin{:});
    end
    function varargout = reducedModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2091, self, varargin{:});
    end
    function varargout = setCOMTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2092, self, varargin{:});
    end
    function varargout = setCOMAsConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2093, self, varargin{:});
    end
    function varargout = setCOMAsConstraintTolerance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2094, self, varargin{:});
    end
    function varargout = isCOMAConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2095, self, varargin{:});
    end
    function varargout = isCOMTargetActive(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2096, self, varargin{:});
    end
    function varargout = deactivateCOMTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2097, self, varargin{:});
    end
    function varargout = setCOMConstraintProjectionDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2098, self, varargin{:});
    end
  end
  methods(Static)
  end
end
