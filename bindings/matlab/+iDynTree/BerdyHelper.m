classdef BerdyHelper < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = BerdyHelper(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1370, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = dynamicTraversal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1371, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1372, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1373, self, varargin{:});
    end
    function varargout = init(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1374, self, varargin{:});
    end
    function varargout = getOptions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1375, self, varargin{:});
    end
    function varargout = getNrOfDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1376, self, varargin{:});
    end
    function varargout = getNrOfDynamicEquations(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1377, self, varargin{:});
    end
    function varargout = getNrOfSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1378, self, varargin{:});
    end
    function varargout = resizeAndZeroBerdyMatrices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1379, self, varargin{:});
    end
    function varargout = getBerdyMatrices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1380, self, varargin{:});
    end
    function varargout = getSensorsOrdering(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1381, self, varargin{:});
    end
    function varargout = getDynamicVariablesOrdering(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1382, self, varargin{:});
    end
    function varargout = serializeDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1383, self, varargin{:});
    end
    function varargout = serializeSensorVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1384, self, varargin{:});
    end
    function varargout = serializeDynamicVariablesComputedFromFixedBaseRNEA(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1385, self, varargin{:});
    end
    function varargout = updateKinematicsFromFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1386, self, varargin{:});
    end
    function varargout = updateKinematicsFromFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1387, self, varargin{:});
    end
    function varargout = updateKinematicsFromTraversalFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1388, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1389, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
