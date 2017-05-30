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
        tmp = iDynTreeMEX(1378, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1379, self);
        self.swigPtr=[];
      end
    end
    function varargout = setModelAndSensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1380, self, varargin{:});
    end
    function varargout = loadModelAndSensorsFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1381, self, varargin{:});
    end
    function varargout = loadModelAndSensorsFromFileWithSpecifiedDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1382, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1383, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1384, self, varargin{:});
    end
    function varargout = submodels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1385, self, varargin{:});
    end
    function varargout = updateKinematicsFromFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1386, self, varargin{:});
    end
    function varargout = updateKinematicsFromFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1387, self, varargin{:});
    end
    function varargout = computeExpectedFTSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1388, self, varargin{:});
    end
    function varargout = estimateExtWrenchesAndJointTorques(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1389, self, varargin{:});
    end
    function varargout = checkThatTheModelIsStill(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1390, self, varargin{:});
    end
    function varargout = estimateLinkNetWrenchesWithoutGravity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1391, self, varargin{:});
    end
  end
  methods(Static)
  end
end
