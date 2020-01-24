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
        tmp = iDynTreeMEX(1764, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = getParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1765, self, varargin{:});
    end
    function varargout = setParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1766, self, varargin{:});
    end
    function varargout = setGravityDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1767, self, varargin{:});
    end
    function varargout = setTimeStepInSeconds(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1768, self, varargin{:});
    end
    function varargout = setBiasCorrelationTimeFactor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1769, self, varargin{:});
    end
    function varargout = useMagnetometerMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1770, self, varargin{:});
    end
    function varargout = setMeasurementNoiseVariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1771, self, varargin{:});
    end
    function varargout = setSystemNoiseVariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1772, self, varargin{:});
    end
    function varargout = setInitialStateCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1773, self, varargin{:});
    end
    function varargout = initializeFilter(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1774, self, varargin{:});
    end
    function varargout = updateFilterWithMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1775, self, varargin{:});
    end
    function varargout = propagateStates(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1776, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsRotationMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1777, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1778, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1779, self, varargin{:});
    end
    function varargout = getInternalStateSize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1780, self, varargin{:});
    end
    function varargout = getInternalState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1781, self, varargin{:});
    end
    function varargout = getDefaultInternalInitialState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1782, self, varargin{:});
    end
    function varargout = setInternalState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1783, self, varargin{:});
    end
    function varargout = setInternalStateInitialOrientation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1784, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1785, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
