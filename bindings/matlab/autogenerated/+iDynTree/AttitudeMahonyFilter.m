classdef AttitudeMahonyFilter < iDynTree.IAttitudeEstimator
  methods
    function self = AttitudeMahonyFilter(varargin)
      self@iDynTree.IAttitudeEstimator(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1773, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = useMagnetoMeterMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1774, self, varargin{:});
    end
    function varargout = setConfidenceForMagnetometerMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1775, self, varargin{:});
    end
    function varargout = setGainkp(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1776, self, varargin{:});
    end
    function varargout = setGainki(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1777, self, varargin{:});
    end
    function varargout = setTimeStepInSeconds(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1778, self, varargin{:});
    end
    function varargout = setGravityDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1779, self, varargin{:});
    end
    function varargout = setParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1780, self, varargin{:});
    end
    function varargout = getParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1781, self, varargin{:});
    end
    function varargout = updateFilterWithMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1782, self, varargin{:});
    end
    function varargout = propagateStates(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1783, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsRotationMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1784, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1785, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1786, self, varargin{:});
    end
    function varargout = getInternalStateSize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1787, self, varargin{:});
    end
    function varargout = getInternalState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1788, self, varargin{:});
    end
    function varargout = getDefaultInternalInitialState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1789, self, varargin{:});
    end
    function varargout = setInternalState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1790, self, varargin{:});
    end
    function varargout = setInternalStateInitialOrientation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1791, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1792, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
