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
        tmp = iDynTreeMEX(1577, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1578, self);
        self.swigPtr=[];
      end
    end
    function varargout = loadRobotAndSensorsModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1579, self, varargin{:});
    end
    function varargout = loadRobotAndSensorsModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1580, self, varargin{:});
    end
    function varargout = loadRegressorStructureFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1581, self, varargin{:});
    end
    function varargout = loadRegressorStructureFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1582, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1583, self, varargin{:});
    end
    function varargout = getNrOfParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1584, self, varargin{:});
    end
    function varargout = getNrOfOutputs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1585, self, varargin{:});
    end
    function varargout = getNrOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1586, self, varargin{:});
    end
    function varargout = getDescriptionOfParameter(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1587, self, varargin{:});
    end
    function varargout = getDescriptionOfParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1588, self, varargin{:});
    end
    function varargout = getDescriptionOfOutput(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1589, self, varargin{:});
    end
    function varargout = getDescriptionOfOutputs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1590, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreeOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1591, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1592, self, varargin{:});
    end
    function varargout = getDescriptionOfLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1593, self, varargin{:});
    end
    function varargout = getDescriptionOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1594, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1595, self, varargin{:});
    end
    function varargout = getNrOfFakeLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1596, self, varargin{:});
    end
    function varargout = getBaseLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1597, self, varargin{:});
    end
    function varargout = getSensorsModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1598, self, varargin{:});
    end
    function varargout = setRobotState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1599, self, varargin{:});
    end
    function varargout = getSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1600, self, varargin{:});
    end
    function varargout = setTorqueSensorMeasurement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1601, self, varargin{:});
    end
    function varargout = computeRegressor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1602, self, varargin{:});
    end
    function varargout = getModelParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1603, self, varargin{:});
    end
    function varargout = computeFloatingBaseIdentifiableSubspace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1604, self, varargin{:});
    end
    function varargout = computeFixedBaseIdentifiableSubspace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1605, self, varargin{:});
    end
    function varargout = generate_random_regressors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1606, self, varargin{:});
    end
  end
  methods(Static)
  end
end
