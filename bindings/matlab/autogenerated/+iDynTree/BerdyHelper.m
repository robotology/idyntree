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
        tmp = iDynTreeMEX(1694, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = dynamicTraversal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1695, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1696, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1697, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1698, self, varargin{:});
    end
    function varargout = init(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1699, self, varargin{:});
    end
    function varargout = getOptions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1700, self, varargin{:});
    end
    function varargout = getNrOfDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1701, self, varargin{:});
    end
    function varargout = getNrOfDynamicEquations(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1702, self, varargin{:});
    end
    function varargout = getNrOfSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1703, self, varargin{:});
    end
    function varargout = resizeAndZeroBerdyMatrices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1704, self, varargin{:});
    end
    function varargout = getBerdyMatrices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1705, self, varargin{:});
    end
    function varargout = getSensorsOrdering(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1706, self, varargin{:});
    end
    function varargout = getRangeSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1707, self, varargin{:});
    end
    function varargout = getRangeDOFSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1708, self, varargin{:});
    end
    function varargout = getRangeJointSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1709, self, varargin{:});
    end
    function varargout = getRangeLinkSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1710, self, varargin{:});
    end
    function varargout = getRangeLinkVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1711, self, varargin{:});
    end
    function varargout = getRangeJointVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1712, self, varargin{:});
    end
    function varargout = getRangeDOFVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1713, self, varargin{:});
    end
    function varargout = getDynamicVariablesOrdering(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1714, self, varargin{:});
    end
    function varargout = serializeDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1715, self, varargin{:});
    end
    function varargout = serializeSensorVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1716, self, varargin{:});
    end
    function varargout = serializeDynamicVariablesComputedFromFixedBaseRNEA(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1717, self, varargin{:});
    end
    function varargout = extractJointTorquesFromDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1718, self, varargin{:});
    end
    function varargout = extractLinkNetExternalWrenchesFromDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1719, self, varargin{:});
    end
    function varargout = updateKinematicsFromFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1720, self, varargin{:});
    end
    function varargout = updateKinematicsFromFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1721, self, varargin{:});
    end
    function varargout = updateKinematicsFromTraversalFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1722, self, varargin{:});
    end
    function varargout = setNetExternalWrenchMeasurementFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1723, self, varargin{:});
    end
    function varargout = getNetExternalWrenchMeasurementFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1724, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1725, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
