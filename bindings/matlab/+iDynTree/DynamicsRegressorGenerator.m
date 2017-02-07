classdef DynamicsRegressorGenerator < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = DynamicsRegressorGenerator(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1455, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1456, self);
        self.swigPtr=[];
      end
    end
    function varargout = loadRobotAndSensorsModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1457, self, varargin{:});
    end
    function varargout = loadRobotAndSensorsModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1458, self, varargin{:});
    end
    function varargout = loadRegressorStructureFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1459, self, varargin{:});
    end
    function varargout = loadRegressorStructureFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1460, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1461, self, varargin{:});
    end
    function varargout = getNrOfParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1462, self, varargin{:});
    end
    function varargout = getNrOfOutputs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1463, self, varargin{:});
    end
    function varargout = getNrOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1464, self, varargin{:});
    end
    function varargout = getDescriptionOfParameter(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1465, self, varargin{:});
    end
    function varargout = getDescriptionOfParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1466, self, varargin{:});
    end
    function varargout = getDescriptionOfOutput(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1467, self, varargin{:});
    end
    function varargout = getDescriptionOfOutputs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1468, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreeOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1469, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1470, self, varargin{:});
    end
    function varargout = getDescriptionOfLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1471, self, varargin{:});
    end
    function varargout = getDescriptionOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1472, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1473, self, varargin{:});
    end
    function varargout = getNrOfFakeLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1474, self, varargin{:});
    end
    function varargout = getBaseLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1475, self, varargin{:});
    end
    function varargout = getSensorsModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1476, self, varargin{:});
    end
    function varargout = setRobotState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1477, self, varargin{:});
    end
    function varargout = getSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1478, self, varargin{:});
    end
    function varargout = setTorqueSensorMeasurement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1479, self, varargin{:});
    end
    function varargout = computeRegressor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1480, self, varargin{:});
    end
    function varargout = getModelParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1481, self, varargin{:});
    end
    function varargout = computeFloatingBaseIdentifiableSubspace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1482, self, varargin{:});
    end
    function varargout = computeFixedBaseIdentifiableSubspace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1483, self, varargin{:});
    end
    function varargout = generate_random_regressors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1484, self, varargin{:});
    end
  end
  methods(Static)
  end
end
