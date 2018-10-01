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
        tmp = iDynTreeMEX(1599, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1600, self);
        self.SwigClear();
      end
    end
    function varargout = setDynamicsConstraintsPriorCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1601, self, varargin{:});
    end
    function varargout = setDynamicsRegularizationPriorCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1602, self, varargin{:});
    end
    function varargout = setDynamicsRegularizationPriorExpectedValue(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1603, self, varargin{:});
    end
    function varargout = setMeasurementsPriorCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1604, self, varargin{:});
    end
    function varargout = dynamicsConstraintsPriorCovarianceInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1605, self, varargin{:});
    end
    function varargout = dynamicsRegularizationPriorCovarianceInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1606, self, varargin{:});
    end
    function varargout = dynamicsRegularizationPriorExpectedValue(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1607, self, varargin{:});
    end
    function varargout = measurementsPriorCovarianceInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1608, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1609, self, varargin{:});
    end
    function varargout = initialize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1610, self, varargin{:});
    end
    function varargout = updateEstimateInformationFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1611, self, varargin{:});
    end
    function varargout = updateEstimateInformationFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1612, self, varargin{:});
    end
    function varargout = doEstimate(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1613, self, varargin{:});
    end
    function varargout = getLastEstimate(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1614, self, varargin{:});
    end
  end
  methods(Static)
  end
end
