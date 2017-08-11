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
        tmp = iDynTreeMEX(1496, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1497, self);
        self.swigPtr=[];
      end
    end
    function varargout = setDynamicsConstraintsPriorCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1498, self, varargin{:});
    end
    function varargout = setDynamicsRegularizationPriorCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1499, self, varargin{:});
    end
    function varargout = setDynamicsRegularizationPriorExpectedValue(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1500, self, varargin{:});
    end
    function varargout = setMeasurementsPriorCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1501, self, varargin{:});
    end
    function varargout = dynamicsConstraintsPriorCovarianceInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1502, self, varargin{:});
    end
    function varargout = dynamicsRegularizationPriorCovarianceInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1503, self, varargin{:});
    end
    function varargout = dynamicsRegularizationPriorExpectedValue(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1504, self, varargin{:});
    end
    function varargout = measurementsPriorCovarianceInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1505, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1506, self, varargin{:});
    end
    function varargout = initialize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1507, self, varargin{:});
    end
    function varargout = updateEstimateInformation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1508, self, varargin{:});
    end
    function varargout = doEstimate(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1509, self, varargin{:});
    end
    function varargout = getLastEstimate(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1510, self, varargin{:});
    end
  end
  methods(Static)
  end
end
