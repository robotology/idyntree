classdef AttitudeQuaternionEKF < iDynTree.IAttitudeEstimator & iDynTree.DiscreteExtendedKalmanFilterHelper
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = AttitudeQuaternionEKF(varargin)
      self@iDynTree.IAttitudeEstimator(iDynTreeSwigRef.Null);
      self@iDynTree.DiscreteExtendedKalmanFilterHelper(iDynTreeSwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1731, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = getParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1732, self, varargin{:});
    end
    function varargout = setParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1733, self, varargin{:});
    end
    function varargout = setGravityDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1734, self, varargin{:});
    end
    function varargout = setTimeStepInSeconds(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1735, self, varargin{:});
    end
    function varargout = setBiasCorrelationTimeFactor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1736, self, varargin{:});
    end
    function varargout = useMagnetometerMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1737, self, varargin{:});
    end
    function varargout = setMeasurementNoiseVariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1738, self, varargin{:});
    end
    function varargout = setSystemNoiseVariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1739, self, varargin{:});
    end
    function varargout = setInitialStateCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1740, self, varargin{:});
    end
    function varargout = initializeFilter(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1741, self, varargin{:});
    end
    function varargout = updateFilterWithMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1742, self, varargin{:});
    end
    function varargout = propagateStates(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1743, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsRotationMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1744, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1745, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1746, self, varargin{:});
    end
    function varargout = getInternalStateSize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1747, self, varargin{:});
    end
    function varargout = getInternalState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1748, self, varargin{:});
    end
    function varargout = getDefaultInternalInitialState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1749, self, varargin{:});
    end
    function varargout = setInternalState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1750, self, varargin{:});
    end
    function varargout = setInternalStateInitialOrientation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1751, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1752, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
