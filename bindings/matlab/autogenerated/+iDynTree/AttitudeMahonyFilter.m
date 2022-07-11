classdef AttitudeMahonyFilter < iDynTree.IAttitudeEstimator
  methods
    function self = AttitudeMahonyFilter(varargin)
      self@iDynTree.IAttitudeEstimator(iDynTreeSwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1666, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = useMagnetoMeterMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1667, self, varargin{:});
    end
    function varargout = setConfidenceForMagnetometerMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1668, self, varargin{:});
    end
    function varargout = setGainkp(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1669, self, varargin{:});
    end
    function varargout = setGainki(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1670, self, varargin{:});
    end
    function varargout = setTimeStepInSeconds(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1671, self, varargin{:});
    end
    function varargout = setGravityDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1672, self, varargin{:});
    end
    function varargout = setParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1673, self, varargin{:});
    end
    function varargout = getParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1674, self, varargin{:});
    end
    function varargout = updateFilterWithMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1675, self, varargin{:});
    end
    function varargout = propagateStates(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1676, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsRotationMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1677, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1678, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1679, self, varargin{:});
    end
    function varargout = getInternalStateSize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1680, self, varargin{:});
    end
    function varargout = getInternalState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1681, self, varargin{:});
    end
    function varargout = getDefaultInternalInitialState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1682, self, varargin{:});
    end
    function varargout = setInternalState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1683, self, varargin{:});
    end
    function varargout = setInternalStateInitialOrientation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1684, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1685, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
