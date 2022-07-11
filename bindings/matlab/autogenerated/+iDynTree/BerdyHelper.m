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
        tmp = iDynTreeMEX(1586, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = dynamicTraversal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1587, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1588, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1589, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1590, self, varargin{:});
    end
    function varargout = init(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1591, self, varargin{:});
    end
    function varargout = getOptions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1592, self, varargin{:});
    end
    function varargout = getNrOfDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1593, self, varargin{:});
    end
    function varargout = getNrOfDynamicEquations(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1594, self, varargin{:});
    end
    function varargout = getNrOfSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1595, self, varargin{:});
    end
    function varargout = resizeAndZeroBerdyMatrices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1596, self, varargin{:});
    end
    function varargout = getBerdyMatrices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1597, self, varargin{:});
    end
    function varargout = getSensorsOrdering(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1598, self, varargin{:});
    end
    function varargout = getRangeSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1599, self, varargin{:});
    end
    function varargout = getRangeDOFSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1600, self, varargin{:});
    end
    function varargout = getRangeJointSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1601, self, varargin{:});
    end
    function varargout = getRangeLinkSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1602, self, varargin{:});
    end
    function varargout = getRangeRCMSensorVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1603, self, varargin{:});
    end
    function varargout = getRangeLinkVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1604, self, varargin{:});
    end
    function varargout = getRangeJointVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1605, self, varargin{:});
    end
    function varargout = getRangeDOFVariable(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1606, self, varargin{:});
    end
    function varargout = getDynamicVariablesOrdering(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1607, self, varargin{:});
    end
    function varargout = serializeDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1608, self, varargin{:});
    end
    function varargout = serializeSensorVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1609, self, varargin{:});
    end
    function varargout = serializeDynamicVariablesComputedFromFixedBaseRNEA(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1610, self, varargin{:});
    end
    function varargout = extractJointTorquesFromDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1611, self, varargin{:});
    end
    function varargout = extractLinkNetExternalWrenchesFromDynamicVariables(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1612, self, varargin{:});
    end
    function varargout = updateKinematicsFromFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1613, self, varargin{:});
    end
    function varargout = updateKinematicsFromFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1614, self, varargin{:});
    end
    function varargout = updateKinematicsFromTraversalFixedBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1615, self, varargin{:});
    end
    function varargout = setNetExternalWrenchMeasurementFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1616, self, varargin{:});
    end
    function varargout = getNetExternalWrenchMeasurementFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1617, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1618, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
