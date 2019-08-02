classdef DiscreteExtendedKalmanFilterHelper < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = ekf_f(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1671, self, varargin{:});
    end
    function varargout = ekf_h(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1672, self, varargin{:});
    end
    function varargout = ekfComputeJacobianF(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1673, self, varargin{:});
    end
    function varargout = ekfComputeJacobianH(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1674, self, varargin{:});
    end
    function varargout = ekfPredict(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1675, self, varargin{:});
    end
    function varargout = ekfUpdate(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1676, self, varargin{:});
    end
    function varargout = ekfInit(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1677, self, varargin{:});
    end
    function varargout = ekfReset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1678, self, varargin{:});
    end
    function varargout = ekfSetMeasurementVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1679, self, varargin{:});
    end
    function varargout = ekfSetInputVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1680, self, varargin{:});
    end
    function varargout = ekfSetInitialState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1681, self, varargin{:});
    end
    function varargout = ekfSetStateCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1682, self, varargin{:});
    end
    function varargout = ekfSetSystemNoiseCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1683, self, varargin{:});
    end
    function varargout = ekfSetMeasurementNoiseCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1684, self, varargin{:});
    end
    function varargout = ekfSetStateSize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1685, self, varargin{:});
    end
    function varargout = ekfSetInputSize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1686, self, varargin{:});
    end
    function varargout = ekfSetOutputSize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1687, self, varargin{:});
    end
    function varargout = ekfGetStates(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1688, self, varargin{:});
    end
    function varargout = ekfGetStateCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1689, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1690, self);
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
