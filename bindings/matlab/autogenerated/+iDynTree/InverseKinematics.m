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
        tmp = iDynTreeMEX(1972, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1973, self);
        self.SwigClear();
      end
    end
    function varargout = loadModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1974, self, varargin{:});
    end
    function varargout = setModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1975, self, varargin{:});
    end
    function varargout = setJointLimits(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1976, self, varargin{:});
    end
    function varargout = getJointLimits(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1977, self, varargin{:});
    end
    function varargout = clearProblem(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1978, self, varargin{:});
    end
    function varargout = setFloatingBaseOnFrameNamed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1979, self, varargin{:});
    end
    function varargout = setCurrentRobotConfiguration(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1980, self, varargin{:});
    end
    function varargout = setJointConfiguration(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1981, self, varargin{:});
    end
    function varargout = setRotationParametrization(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1982, self, varargin{:});
    end
    function varargout = rotationParametrization(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1983, self, varargin{:});
    end
    function varargout = setMaxIterations(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1984, self, varargin{:});
    end
    function varargout = maxIterations(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1985, self, varargin{:});
    end
    function varargout = setMaxCPUTime(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1986, self, varargin{:});
    end
    function varargout = maxCPUTime(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1987, self, varargin{:});
    end
    function varargout = setCostTolerance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1988, self, varargin{:});
    end
    function varargout = costTolerance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1989, self, varargin{:});
    end
    function varargout = setConstraintsTolerance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1990, self, varargin{:});
    end
    function varargout = constraintsTolerance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1991, self, varargin{:});
    end
    function varargout = setVerbosity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1992, self, varargin{:});
    end
    function varargout = linearSolverName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1993, self, varargin{:});
    end
    function varargout = setLinearSolverName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1994, self, varargin{:});
    end
    function varargout = addFrameConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1995, self, varargin{:});
    end
    function varargout = addFramePositionConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1996, self, varargin{:});
    end
    function varargout = addFrameRotationConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1997, self, varargin{:});
    end
    function varargout = activateFrameConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1998, self, varargin{:});
    end
    function varargout = deactivateFrameConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1999, self, varargin{:});
    end
    function varargout = isFrameConstraintActive(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2000, self, varargin{:});
    end
    function varargout = addCenterOfMassProjectionConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2001, self, varargin{:});
    end
    function varargout = getCenterOfMassProjectionMargin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2002, self, varargin{:});
    end
    function varargout = getCenterOfMassProjectConstraintConvexHull(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2003, self, varargin{:});
    end
    function varargout = addTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2004, self, varargin{:});
    end
    function varargout = addPositionTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2005, self, varargin{:});
    end
    function varargout = addRotationTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2006, self, varargin{:});
    end
    function varargout = updateTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2007, self, varargin{:});
    end
    function varargout = updatePositionTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2008, self, varargin{:});
    end
    function varargout = updateRotationTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2009, self, varargin{:});
    end
    function varargout = setDefaultTargetResolutionMode(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2010, self, varargin{:});
    end
    function varargout = defaultTargetResolutionMode(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2011, self, varargin{:});
    end
    function varargout = setTargetResolutionMode(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2012, self, varargin{:});
    end
    function varargout = targetResolutionMode(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2013, self, varargin{:});
    end
    function varargout = setDesiredFullJointsConfiguration(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2014, self, varargin{:});
    end
    function varargout = setDesiredReducedJointConfiguration(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2015, self, varargin{:});
    end
    function varargout = setFullJointsInitialCondition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2016, self, varargin{:});
    end
    function varargout = setReducedInitialCondition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2017, self, varargin{:});
    end
    function varargout = solve(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2018, self, varargin{:});
    end
    function varargout = getFullJointsSolution(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2019, self, varargin{:});
    end
    function varargout = getReducedSolution(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2020, self, varargin{:});
    end
    function varargout = getPoseForFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2021, self, varargin{:});
    end
    function varargout = fullModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2022, self, varargin{:});
    end
    function varargout = reducedModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2023, self, varargin{:});
    end
    function varargout = setCOMTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2024, self, varargin{:});
    end
    function varargout = setCOMAsConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2025, self, varargin{:});
    end
    function varargout = setCOMAsConstraintTolerance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2026, self, varargin{:});
    end
    function varargout = isCOMAConstraint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2027, self, varargin{:});
    end
    function varargout = isCOMTargetActive(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2028, self, varargin{:});
    end
    function varargout = deactivateCOMTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2029, self, varargin{:});
    end
    function varargout = setCOMConstraintProjectionDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2030, self, varargin{:});
    end
  end
  methods(Static)
  end
end
