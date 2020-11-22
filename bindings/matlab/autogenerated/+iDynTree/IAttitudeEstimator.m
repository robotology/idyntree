classdef IAttitudeEstimator < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1597, self);
        self.SwigClear();
      end
    end
    function varargout = updateFilterWithMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1598, self, varargin{:});
    end
    function varargout = propagateStates(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1599, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsRotationMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1600, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1601, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1602, self, varargin{:});
    end
    function varargout = getInternalStateSize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1603, self, varargin{:});
    end
    function varargout = getInternalState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1604, self, varargin{:});
    end
    function varargout = getDefaultInternalInitialState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1605, self, varargin{:});
    end
    function varargout = setInternalState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1606, self, varargin{:});
    end
    function varargout = setInternalStateInitialOrientation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1607, self, varargin{:});
    end
    function self = IAttitudeEstimator(varargin)
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
