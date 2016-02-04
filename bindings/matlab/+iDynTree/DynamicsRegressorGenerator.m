classdef DynamicsRegressorGenerator < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = DynamicsRegressorGenerator(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(996, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(997, self);
        self.swigPtr=[];
      end
    end
    function varargout = loadRobotAndSensorsModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(998, self, varargin{:});
    end
    function varargout = loadRobotAndSensorsModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(999, self, varargin{:});
    end
    function varargout = loadRegressorStructureFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1000, self, varargin{:});
    end
    function varargout = loadRegressorStructureFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1001, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1002, self, varargin{:});
    end
    function varargout = getNrOfParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1003, self, varargin{:});
    end
    function varargout = getNrOfOutputs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1004, self, varargin{:});
    end
    function varargout = getNrOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1005, self, varargin{:});
    end
    function varargout = getDescriptionOfParameter(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1006, self, varargin{:});
    end
    function varargout = getDescriptionOfParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1007, self, varargin{:});
    end
    function varargout = getDescriptionOfOutput(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1008, self, varargin{:});
    end
    function varargout = getDescriptionOfOutputs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1009, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreeOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1010, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1011, self, varargin{:});
    end
    function varargout = getBaseLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1012, self, varargin{:});
    end
    function varargout = getSensorsModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1013, self, varargin{:});
    end
    function varargout = setRobotState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1014, self, varargin{:});
    end
    function varargout = getSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1015, self, varargin{:});
    end
    function varargout = computeRegressor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1016, self, varargin{:});
    end
    function varargout = getModelParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1017, self, varargin{:});
    end
    function varargout = computeFloatingBaseIdentifiableSubspace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1018, self, varargin{:});
    end
    function varargout = computeFixedBaseIdentifiableSubspace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1019, self, varargin{:});
    end
  end
  methods(Static)
  end
end
