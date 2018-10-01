classdef ExtWrenchesAndJointTorquesEstimator < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = ExtWrenchesAndJointTorquesEstimator(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1500, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1501, self);
        self.SwigClear();
      end
    end
    function varargout = setModelAndSensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1502, self, varargin{:});
    end
    function varargout = loadModelAndSensorsFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1503, self, varargin{:});
    end
    function varargout = loadModelAndSensorsFromFileWithSpecifiedDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1504, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1505, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1506, self, varargin{:});
    end
    function varargout = submodels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1507, self, varargin{:});
    end
    function varargout = updateKinematicsFromFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1508, self, varargin{:});
    end
    function varargout = updateKinematicsFromFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1509, self, varargin{:});
    end
    function varargout = computeExpectedFTSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1510, self, varargin{:});
    end
    function varargout = estimateExtWrenchesAndJointTorques(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1511, self, varargin{:});
    end
    function varargout = checkThatTheModelIsStill(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1512, self, varargin{:});
    end
    function varargout = estimateLinkNetWrenchesWithoutGravity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1513, self, varargin{:});
    end
  end
  methods(Static)
  end
end
