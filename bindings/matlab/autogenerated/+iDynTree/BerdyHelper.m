classdef BerdyHelper < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = BerdyHelper(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1630, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = dynamicTraversal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1631, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1632, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1633, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1634, self, varargin{:});
    end
    function varargout = init(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1635, self, varargin{:});
    end
    function varargout = getOptions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1636, self, varargin{:});
    end
    function varargout = getNrOfDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1637, self, varargin{:});
    end
    function varargout = getNrOfDynamicEquations(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1638, self, varargin{:});
    end
    function varargout = getNrOfSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1639, self, varargin{:});
    end
    function varargout = resizeAndZeroBerdyMatrices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1640, self, varargin{:});
    end
    function varargout = getBerdyMatrices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1641, self, varargin{:});
    end
    function varargout = getSensorsOrdering(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1642, self, varargin{:});
    end
    function varargout = getRangeSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1643, self, varargin{:});
    end
    function varargout = getRangeDOFSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1644, self, varargin{:});
    end
    function varargout = getRangeJointSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1645, self, varargin{:});
    end
    function varargout = getRangeLinkSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1646, self, varargin{:});
    end
    function varargout = getRangeRCMSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1647, self, varargin{:});
    end
    function varargout = getRangeLinkVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1648, self, varargin{:});
    end
    function varargout = getRangeJointVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1649, self, varargin{:});
    end
    function varargout = getRangeDOFVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1650, self, varargin{:});
    end
    function varargout = getDynamicVariablesOrdering(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1651, self, varargin{:});
    end
    function varargout = serializeDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1652, self, varargin{:});
    end
    function varargout = serializeSensorVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1653, self, varargin{:});
    end
    function varargout = serializeDynamicVariablesComputedFromFixedBaseRNEA(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1654, self, varargin{:});
    end
    function varargout = extractJointTorquesFromDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1655, self, varargin{:});
    end
    function varargout = extractLinkNetExternalWrenchesFromDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1656, self, varargin{:});
    end
    function varargout = updateKinematicsFromFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1657, self, varargin{:});
    end
    function varargout = updateKinematicsFromFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1658, self, varargin{:});
    end
    function varargout = updateKinematicsFromTraversalFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1659, self, varargin{:});
    end
    function varargout = setNetExternalWrenchMeasurementFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1660, self, varargin{:});
    end
    function varargout = getNetExternalWrenchMeasurementFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1661, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1662, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
