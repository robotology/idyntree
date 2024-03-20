classdef ExtWrenchesAndJointTorquesEstimator < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = ExtWrenchesAndJointTorquesEstimator(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1616, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1617, self);
        self.SwigClear();
      end
    end
    function varargout = setModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1618, self, varargin{:});
    end
    function varargout = setModelAndSensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1619, self, varargin{:});
    end
    function varargout = loadModelAndSensorsFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1620, self, varargin{:});
    end
    function varargout = loadModelAndSensorsFromFileWithSpecifiedDOFs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1621, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1622, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1623, self, varargin{:});
    end
    function varargout = submodels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1624, self, varargin{:});
    end
    function varargout = updateKinematicsFromFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1625, self, varargin{:});
    end
    function varargout = updateKinematicsFromFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1626, self, varargin{:});
    end
    function varargout = computeExpectedFTSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1627, self, varargin{:});
    end
    function varargout = computeSubModelMatrixRelatingFTSensorsMeasuresAndKinematics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1628, self, varargin{:});
    end
    function varargout = estimateExtWrenchesAndJointTorques(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1629, self, varargin{:});
    end
    function varargout = checkThatTheModelIsStill(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1630, self, varargin{:});
    end
    function varargout = estimateLinkNetWrenchesWithoutGravity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1631, self, varargin{:});
    end
  end
  methods(Static)
  end
end
