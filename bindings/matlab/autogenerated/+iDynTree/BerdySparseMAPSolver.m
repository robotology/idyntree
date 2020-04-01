classdef BerdySparseMAPSolver < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = BerdySparseMAPSolver(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1726, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1727, self);
        self.SwigClear();
      end
    end
    function varargout = setDynamicsConstraintsPriorCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1728, self, varargin{:});
    end
    function varargout = setDynamicsRegularizationPriorCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1729, self, varargin{:});
    end
    function varargout = setDynamicsRegularizationPriorExpectedValue(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1730, self, varargin{:});
    end
    function varargout = setMeasurementsPriorCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1731, self, varargin{:});
    end
    function varargout = dynamicsConstraintsPriorCovarianceInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1732, self, varargin{:});
    end
    function varargout = dynamicsRegularizationPriorCovarianceInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1733, self, varargin{:});
    end
    function varargout = dynamicsRegularizationPriorExpectedValue(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1734, self, varargin{:});
    end
    function varargout = measurementsPriorCovarianceInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1735, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1736, self, varargin{:});
    end
    function varargout = initialize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1737, self, varargin{:});
    end
    function varargout = updateEstimateInformationFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1738, self, varargin{:});
    end
    function varargout = updateEstimateInformationFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1739, self, varargin{:});
    end
    function varargout = doEstimate(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1740, self, varargin{:});
    end
    function varargout = getLastEstimate(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1741, self, varargin{:});
    end
  end
  methods(Static)
  end
end
