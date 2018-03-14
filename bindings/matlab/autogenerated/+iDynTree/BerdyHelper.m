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
        tmp = iDynTreeMEX(1509, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = dynamicTraversal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1510, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1511, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1512, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1513, self, varargin{:});
    end
    function varargout = init(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1514, self, varargin{:});
    end
    function varargout = getOptions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1515, self, varargin{:});
    end
    function varargout = getNrOfDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1516, self, varargin{:});
    end
    function varargout = getNrOfDynamicEquations(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1517, self, varargin{:});
    end
    function varargout = getNrOfSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1518, self, varargin{:});
    end
    function varargout = resizeAndZeroBerdyMatrices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1519, self, varargin{:});
    end
    function varargout = getBerdyMatrices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1520, self, varargin{:});
    end
    function varargout = getSensorsOrdering(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1521, self, varargin{:});
    end
    function varargout = getRangeSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1522, self, varargin{:});
    end
    function varargout = getRangeDOFSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1523, self, varargin{:});
    end
    function varargout = getRangeJointSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1524, self, varargin{:});
    end
    function varargout = getRangeLinkSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1525, self, varargin{:});
    end
    function varargout = getRangeLinkVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1526, self, varargin{:});
    end
    function varargout = getRangeJointVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1527, self, varargin{:});
    end
    function varargout = getRangeDOFVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1528, self, varargin{:});
    end
    function varargout = getDynamicVariablesOrdering(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1529, self, varargin{:});
    end
    function varargout = serializeDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1530, self, varargin{:});
    end
    function varargout = serializeSensorVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1531, self, varargin{:});
    end
    function varargout = serializeDynamicVariablesComputedFromFixedBaseRNEA(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1532, self, varargin{:});
    end
    function varargout = extractJointTorquesFromDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1533, self, varargin{:});
    end
    function varargout = extractLinkNetExternalWrenchesFromDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1534, self, varargin{:});
    end
    function varargout = updateKinematicsFromFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1535, self, varargin{:});
    end
    function varargout = updateKinematicsFromFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1536, self, varargin{:});
    end
    function varargout = updateKinematicsFromTraversalFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1537, self, varargin{:});
    end
    function varargout = setNetExternalWrenchMeasurementFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1538, self, varargin{:});
    end
    function varargout = getNetExternalWrenchMeasurementFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1539, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1540, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
