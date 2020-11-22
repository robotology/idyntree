classdef InverseKinematics < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = InverseKinematics(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1930, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1931, self);
        self.SwigClear();
      end
    end
    function varargout = loadModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1932, self, varargin{:});
    end
    function varargout = setModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1933, self, varargin{:});
    end
    function varargout = setJointLimits(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1934, self, varargin{:});
    end
    function varargout = getJointLimits(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1935, self, varargin{:});
    end
    function varargout = clearProblem(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1936, self, varargin{:});
    end
    function varargout = setFloatingBaseOnFrameNamed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1937, self, varargin{:});
    end
    function varargout = setCurrentRobotConfiguration(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1938, self, varargin{:});
    end
    function varargout = setJointConfiguration(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1939, self, varargin{:});
    end
    function varargout = setRotationParametrization(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1940, self, varargin{:});
    end
    function varargout = rotationParametrization(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1941, self, varargin{:});
    end
    function varargout = setMaxIterations(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1942, self, varargin{:});
    end
    function varargout = maxIterations(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1943, self, varargin{:});
    end
    function varargout = setMaxCPUTime(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1944, self, varargin{:});
    end
    function varargout = maxCPUTime(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1945, self, varargin{:});
    end
    function varargout = setCostTolerance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1946, self, varargin{:});
    end
    function varargout = costTolerance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1947, self, varargin{:});
    end
    function varargout = setConstraintsTolerance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1948, self, varargin{:});
    end
    function varargout = constraintsTolerance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1949, self, varargin{:});
    end
    function varargout = setVerbosity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1950, self, varargin{:});
    end
    function varargout = linearSolverName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1951, self, varargin{:});
    end
    function varargout = setLinearSolverName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1952, self, varargin{:});
    end
    function varargout = addFrameConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1953, self, varargin{:});
    end
    function varargout = addFramePositionConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1954, self, varargin{:});
    end
    function varargout = addFrameRotationConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1955, self, varargin{:});
    end
    function varargout = activateFrameConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1956, self, varargin{:});
    end
    function varargout = deactivateFrameConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1957, self, varargin{:});
    end
    function varargout = isFrameConstraintActive(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1958, self, varargin{:});
    end
    function varargout = addCenterOfMassProjectionConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1959, self, varargin{:});
    end
    function varargout = getCenterOfMassProjectionMargin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1960, self, varargin{:});
    end
    function varargout = getCenterOfMassProjectConstraintConvexHull(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1961, self, varargin{:});
    end
    function varargout = addTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1962, self, varargin{:});
    end
    function varargout = addPositionTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1963, self, varargin{:});
    end
    function varargout = addRotationTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1964, self, varargin{:});
    end
    function varargout = updateTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1965, self, varargin{:});
    end
    function varargout = updatePositionTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1966, self, varargin{:});
    end
    function varargout = updateRotationTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1967, self, varargin{:});
    end
    function varargout = setDefaultTargetResolutionMode(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1968, self, varargin{:});
    end
    function varargout = defaultTargetResolutionMode(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1969, self, varargin{:});
    end
    function varargout = setTargetResolutionMode(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1970, self, varargin{:});
    end
    function varargout = targetResolutionMode(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1971, self, varargin{:});
    end
    function varargout = setDesiredFullJointsConfiguration(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1972, self, varargin{:});
    end
    function varargout = setDesiredReducedJointConfiguration(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1973, self, varargin{:});
    end
    function varargout = setFullJointsInitialCondition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1974, self, varargin{:});
    end
    function varargout = setReducedInitialCondition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1975, self, varargin{:});
    end
    function varargout = solve(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1976, self, varargin{:});
    end
    function varargout = getFullJointsSolution(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1977, self, varargin{:});
    end
    function varargout = getReducedSolution(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1978, self, varargin{:});
    end
    function varargout = getPoseForFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1979, self, varargin{:});
    end
    function varargout = fullModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1980, self, varargin{:});
    end
    function varargout = reducedModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1981, self, varargin{:});
    end
    function varargout = setCOMTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1982, self, varargin{:});
    end
    function varargout = setCOMAsConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1983, self, varargin{:});
    end
    function varargout = setCOMAsConstraintTolerance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1984, self, varargin{:});
    end
    function varargout = isCOMAConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1985, self, varargin{:});
    end
    function varargout = isCOMTargetActive(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1986, self, varargin{:});
    end
    function varargout = deactivateCOMTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1987, self, varargin{:});
    end
    function varargout = setCOMConstraintProjectionDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1988, self, varargin{:});
    end
  end
  methods(Static)
  end
end
