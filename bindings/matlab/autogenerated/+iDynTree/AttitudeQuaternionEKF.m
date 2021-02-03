classdef AttitudeQuaternionEKF < iDynTree.IAttitudeEstimator & iDynTree.DiscreteExtendedKalmanFilterHelper
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = AttitudeQuaternionEKF(varargin)
      self@iDynTree.IAttitudeEstimator(SwigRef.Null);
      self@iDynTree.DiscreteExtendedKalmanFilterHelper(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1687, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = getParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1688, self, varargin{:});
    end
    function varargout = setParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1689, self, varargin{:});
    end
    function varargout = setGravityDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1690, self, varargin{:});
    end
    function varargout = setTimeStepInSeconds(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1691, self, varargin{:});
    end
    function varargout = setBiasCorrelationTimeFactor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1692, self, varargin{:});
    end
    function varargout = useMagnetometerMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1693, self, varargin{:});
    end
    function varargout = setMeasurementNoiseVariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1694, self, varargin{:});
    end
    function varargout = setSystemNoiseVariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1695, self, varargin{:});
    end
    function varargout = setInitialStateCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1696, self, varargin{:});
    end
    function varargout = initializeFilter(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1697, self, varargin{:});
    end
    function varargout = updateFilterWithMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1698, self, varargin{:});
    end
    function varargout = propagateStates(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1699, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsRotationMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1700, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1701, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1702, self, varargin{:});
    end
    function varargout = getInternalStateSize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1703, self, varargin{:});
    end
    function varargout = getInternalState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1704, self, varargin{:});
    end
    function varargout = getDefaultInternalInitialState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1705, self, varargin{:});
    end
    function varargout = setInternalState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1706, self, varargin{:});
    end
    function varargout = setInternalStateInitialOrientation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1707, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1708, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
