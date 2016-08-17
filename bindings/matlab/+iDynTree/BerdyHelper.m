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
        tmp = iDynTreeMEX(1238, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = dynamicTraversal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1239, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1240, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1241, self, varargin{:});
    end
    function varargout = init(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1242, self, varargin{:});
    end
    function varargout = getOptions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1243, self, varargin{:});
    end
    function varargout = getNrOfDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1244, self, varargin{:});
    end
    function varargout = getNrOfDynamicEquations(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1245, self, varargin{:});
    end
    function varargout = getNrOfSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1246, self, varargin{:});
    end
    function varargout = resizeAndZeroBerdyMatrices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1247, self, varargin{:});
    end
    function varargout = getBerdyMatrices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1248, self, varargin{:});
    end
    function varargout = getSensorsOrdering(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1249, self, varargin{:});
    end
    function varargout = getDynamicVariablesOrdering(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1250, self, varargin{:});
    end
    function varargout = serializeDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1251, self, varargin{:});
    end
    function varargout = serializeSensorVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1252, self, varargin{:});
    end
    function varargout = serializeDynamicVariablesComputedFromFixedBaseRNEA(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1253, self, varargin{:});
    end
    function varargout = updateKinematicsFromFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1254, self, varargin{:});
    end
    function varargout = updateKinematicsFromFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1255, self, varargin{:});
    end
    function varargout = updateKinematicsFromTraversalFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1256, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1257, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
