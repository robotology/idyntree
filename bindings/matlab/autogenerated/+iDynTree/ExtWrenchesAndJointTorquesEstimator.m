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
        tmp = iDynTreeMEX(1458, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1459, self);
        self.SwigClear();
      end
    end
    function varargout = setModelAndSensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1460, self, varargin{:});
    end
    function varargout = loadModelAndSensorsFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1461, self, varargin{:});
    end
    function varargout = loadModelAndSensorsFromFileWithSpecifiedDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1462, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1463, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1464, self, varargin{:});
    end
    function varargout = submodels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1465, self, varargin{:});
    end
    function varargout = updateKinematicsFromFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1466, self, varargin{:});
    end
    function varargout = updateKinematicsFromFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1467, self, varargin{:});
    end
    function varargout = computeExpectedFTSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1468, self, varargin{:});
    end
    function varargout = estimateExtWrenchesAndJointTorques(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1469, self, varargin{:});
    end
    function varargout = checkThatTheModelIsStill(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1470, self, varargin{:});
    end
    function varargout = estimateLinkNetWrenchesWithoutGravity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1471, self, varargin{:});
    end
  end
  methods(Static)
  end
end
