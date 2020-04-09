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
        tmp = iDynTreeMEX(1863, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = getParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1864, self, varargin{:});
    end
    function varargout = setParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1865, self, varargin{:});
    end
    function varargout = setGravityDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1866, self, varargin{:});
    end
    function varargout = setTimeStepInSeconds(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1867, self, varargin{:});
    end
    function varargout = setBiasCorrelationTimeFactor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1868, self, varargin{:});
    end
    function varargout = useMagnetometerMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1869, self, varargin{:});
    end
    function varargout = setMeasurementNoiseVariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1870, self, varargin{:});
    end
    function varargout = setSystemNoiseVariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1871, self, varargin{:});
    end
    function varargout = setInitialStateCovariance(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1872, self, varargin{:});
    end
    function varargout = initializeFilter(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1873, self, varargin{:});
    end
    function varargout = updateFilterWithMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1874, self, varargin{:});
    end
    function varargout = propagateStates(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1875, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsRotationMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1876, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1877, self, varargin{:});
    end
    function varargout = getOrientationEstimateAsRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1878, self, varargin{:});
    end
    function varargout = getInternalStateSize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1879, self, varargin{:});
    end
    function varargout = getInternalState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1880, self, varargin{:});
    end
    function varargout = getDefaultInternalInitialState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1881, self, varargin{:});
    end
    function varargout = setInternalState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1882, self, varargin{:});
    end
    function varargout = setInternalStateInitialOrientation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1883, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1884, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
