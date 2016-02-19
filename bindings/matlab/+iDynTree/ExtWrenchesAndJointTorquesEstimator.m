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
        tmp = iDynTreeMEX(1000, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1001, self);
        self.swigPtr=[];
      end
    end
    function varargout = setModelAndSensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1002, self, varargin{:});
    end
    function varargout = loadModelAndSensorsFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1003, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1004, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1005, self, varargin{:});
    end
    function varargout = updateKinematicsFromFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1006, self, varargin{:});
    end
    function varargout = updateKinematicsFromFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1007, self, varargin{:});
    end
    function varargout = computeExpectedFTSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1008, self, varargin{:});
    end
    function varargout = estimateExtWrenchesAndJointTorques(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1009, self, varargin{:});
    end
  end
  methods(Static)
  end
end
