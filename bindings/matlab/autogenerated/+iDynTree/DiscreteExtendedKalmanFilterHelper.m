classdef DiscreteExtendedKalmanFilterHelper < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = ekf_f(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1719, self, varargin{:});
    end
    function varargout = ekf_h(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1720, self, varargin{:});
    end
    function varargout = ekfComputeJacobianF(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1721, self, varargin{:});
    end
    function varargout = ekfComputeJacobianH(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1722, self, varargin{:});
    end
    function varargout = ekfPredict(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1723, self, varargin{:});
    end
    function varargout = ekfUpdate(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1724, self, varargin{:});
    end
    function varargout = ekfInit(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1725, self, varargin{:});
    end
    function varargout = ekfReset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1726, self, varargin{:});
    end
    function varargout = ekfSetMeasurementVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1727, self, varargin{:});
    end
    function varargout = ekfSetInputVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1728, self, varargin{:});
    end
    function varargout = ekfSetInitialState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1729, self, varargin{:});
    end
    function varargout = ekfSetStateCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1730, self, varargin{:});
    end
    function varargout = ekfSetSystemNoiseCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1731, self, varargin{:});
    end
    function varargout = ekfSetMeasurementNoiseCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1732, self, varargin{:});
    end
    function varargout = ekfSetStateSize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1733, self, varargin{:});
    end
    function varargout = ekfSetInputSize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1734, self, varargin{:});
    end
    function varargout = ekfSetOutputSize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1735, self, varargin{:});
    end
    function varargout = ekfGetStates(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1736, self, varargin{:});
    end
    function varargout = ekfGetStateCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1737, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1738, self);
        self.SwigClear();
      end
    end
    function self = DiscreteExtendedKalmanFilterHelper(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end
