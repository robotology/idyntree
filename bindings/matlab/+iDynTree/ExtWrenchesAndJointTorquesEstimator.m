classdef ExtWrenchesAndJointTorquesEstimator < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = ExtWrenchesAndJointTorquesEstimator(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(965, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(966, self);
        self.swigPtr=[];
      end
    end
    function varargout = setModelAndSensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(967, self, varargin{:});
    end
    function varargout = loadModelAndSensorsFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(968, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(969, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(970, self, varargin{:});
    end
    function varargout = updateKinematicsFromFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(971, self, varargin{:});
    end
    function varargout = updateKinematicsFromFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(972, self, varargin{:});
    end
    function varargout = computeExpectedFTSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(973, self, varargin{:});
    end
    function varargout = estimateExtWrenchesAndJointTorques(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(974, self, varargin{:});
    end
  end
  methods(Static)
  end
end
