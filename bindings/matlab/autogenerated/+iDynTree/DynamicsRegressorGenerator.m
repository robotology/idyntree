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
        tmp = iDynTreeMEX(1881, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1882, self);
        self.SwigClear();
      end
    end
    function varargout = loadRobotAndSensorsModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1883, self, varargin{:});
    end
    function varargout = loadRobotAndSensorsModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1884, self, varargin{:});
    end
    function varargout = loadRegressorStructureFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1885, self, varargin{:});
    end
    function varargout = loadRegressorStructureFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1886, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1887, self, varargin{:});
    end
    function varargout = getNrOfParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1888, self, varargin{:});
    end
    function varargout = getNrOfOutputs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1889, self, varargin{:});
    end
    function varargout = getNrOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1890, self, varargin{:});
    end
    function varargout = getDescriptionOfParameter(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1891, self, varargin{:});
    end
    function varargout = getDescriptionOfParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1892, self, varargin{:});
    end
    function varargout = getDescriptionOfOutput(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1893, self, varargin{:});
    end
    function varargout = getDescriptionOfOutputs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1894, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreeOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1895, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1896, self, varargin{:});
    end
    function varargout = getDescriptionOfLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1897, self, varargin{:});
    end
    function varargout = getDescriptionOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1898, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1899, self, varargin{:});
    end
    function varargout = getNrOfFakeLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1900, self, varargin{:});
    end
    function varargout = getBaseLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1901, self, varargin{:});
    end
    function varargout = getSensorsModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1902, self, varargin{:});
    end
    function varargout = setRobotState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1903, self, varargin{:});
    end
    function varargout = getSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1904, self, varargin{:});
    end
    function varargout = setTorqueSensorMeasurement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1905, self, varargin{:});
    end
    function varargout = computeRegressor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1906, self, varargin{:});
    end
    function varargout = getModelParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1907, self, varargin{:});
    end
    function varargout = computeFloatingBaseIdentifiableSubspace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1908, self, varargin{:});
    end
    function varargout = computeFixedBaseIdentifiableSubspace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1909, self, varargin{:});
    end
    function varargout = generate_random_regressors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1910, self, varargin{:});
    end
  end
  methods(Static)
  end
end
