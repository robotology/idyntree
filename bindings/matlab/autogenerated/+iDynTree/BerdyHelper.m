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
        tmp = iDynTreeMEX(1719, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = dynamicTraversal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1720, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1721, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1722, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1723, self, varargin{:});
    end
    function varargout = init(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1724, self, varargin{:});
    end
    function varargout = getOptions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1725, self, varargin{:});
    end
    function varargout = getNrOfDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1726, self, varargin{:});
    end
    function varargout = getNrOfDynamicEquations(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1727, self, varargin{:});
    end
    function varargout = getNrOfSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1728, self, varargin{:});
    end
    function varargout = resizeAndZeroBerdyMatrices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1729, self, varargin{:});
    end
    function varargout = getBerdyMatrices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1730, self, varargin{:});
    end
    function varargout = getSensorsOrdering(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1731, self, varargin{:});
    end
    function varargout = getRangeSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1732, self, varargin{:});
    end
    function varargout = getRangeDOFSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1733, self, varargin{:});
    end
    function varargout = getRangeJointSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1734, self, varargin{:});
    end
    function varargout = getRangeLinkSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1735, self, varargin{:});
    end
    function varargout = getRangeLinkVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1736, self, varargin{:});
    end
    function varargout = getRangeJointVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1737, self, varargin{:});
    end
    function varargout = getRangeDOFVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1738, self, varargin{:});
    end
    function varargout = getDynamicVariablesOrdering(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1739, self, varargin{:});
    end
    function varargout = serializeDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1740, self, varargin{:});
    end
    function varargout = serializeSensorVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1741, self, varargin{:});
    end
    function varargout = serializeDynamicVariablesComputedFromFixedBaseRNEA(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1742, self, varargin{:});
    end
    function varargout = extractJointTorquesFromDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1743, self, varargin{:});
    end
    function varargout = extractLinkNetExternalWrenchesFromDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1744, self, varargin{:});
    end
    function varargout = updateKinematicsFromFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1745, self, varargin{:});
    end
    function varargout = updateKinematicsFromFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1746, self, varargin{:});
    end
    function varargout = updateKinematicsFromTraversalFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1747, self, varargin{:});
    end
    function varargout = setNetExternalWrenchMeasurementFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1748, self, varargin{:});
    end
    function varargout = getNetExternalWrenchMeasurementFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1749, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1750, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
