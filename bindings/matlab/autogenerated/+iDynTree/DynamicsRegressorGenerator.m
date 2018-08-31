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
        tmp = iDynTreeMEX(1624, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1625, self);
        self.SwigClear();
      end
    end
    function varargout = loadRobotAndSensorsModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1626, self, varargin{:});
    end
    function varargout = loadRobotAndSensorsModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1627, self, varargin{:});
    end
    function varargout = loadRegressorStructureFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1628, self, varargin{:});
    end
    function varargout = loadRegressorStructureFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1629, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1630, self, varargin{:});
    end
    function varargout = getNrOfParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1631, self, varargin{:});
    end
    function varargout = getNrOfOutputs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1632, self, varargin{:});
    end
    function varargout = getNrOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1633, self, varargin{:});
    end
    function varargout = getDescriptionOfParameter(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1634, self, varargin{:});
    end
    function varargout = getDescriptionOfParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1635, self, varargin{:});
    end
    function varargout = getDescriptionOfOutput(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1636, self, varargin{:});
    end
    function varargout = getDescriptionOfOutputs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1637, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreeOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1638, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1639, self, varargin{:});
    end
    function varargout = getDescriptionOfLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1640, self, varargin{:});
    end
    function varargout = getDescriptionOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1641, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1642, self, varargin{:});
    end
    function varargout = getNrOfFakeLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1643, self, varargin{:});
    end
    function varargout = getBaseLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1644, self, varargin{:});
    end
    function varargout = getSensorsModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1645, self, varargin{:});
    end
    function varargout = setRobotState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1646, self, varargin{:});
    end
    function varargout = getSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1647, self, varargin{:});
    end
    function varargout = setTorqueSensorMeasurement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1648, self, varargin{:});
    end
    function varargout = computeRegressor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1649, self, varargin{:});
    end
    function varargout = getModelParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1650, self, varargin{:});
    end
    function varargout = computeFloatingBaseIdentifiableSubspace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1651, self, varargin{:});
    end
    function varargout = computeFixedBaseIdentifiableSubspace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1652, self, varargin{:});
    end
    function varargout = generate_random_regressors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1653, self, varargin{:});
    end
  end
  methods(Static)
  end
end
