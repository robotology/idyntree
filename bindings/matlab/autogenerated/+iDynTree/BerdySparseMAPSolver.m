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
        tmp = iDynTreeMEX(1573, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1574, self);
        self.SwigClear();
      end
    end
    function varargout = setDynamicsConstraintsPriorCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1575, self, varargin{:});
    end
    function varargout = setDynamicsRegularizationPriorCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1576, self, varargin{:});
    end
    function varargout = setDynamicsRegularizationPriorExpectedValue(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1577, self, varargin{:});
    end
    function varargout = setMeasurementsPriorCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1578, self, varargin{:});
    end
    function varargout = dynamicsConstraintsPriorCovarianceInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1579, self, varargin{:});
    end
    function varargout = dynamicsRegularizationPriorCovarianceInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1580, self, varargin{:});
    end
    function varargout = dynamicsRegularizationPriorExpectedValue(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1581, self, varargin{:});
    end
    function varargout = measurementsPriorCovarianceInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1582, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1583, self, varargin{:});
    end
    function varargout = initialize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1584, self, varargin{:});
    end
    function varargout = updateEstimateInformationFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1585, self, varargin{:});
    end
    function varargout = updateEstimateInformationFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1586, self, varargin{:});
    end
    function varargout = doEstimate(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1587, self, varargin{:});
    end
    function varargout = getLastEstimate(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1588, self, varargin{:});
    end
  end
  methods(Static)
  end
end
