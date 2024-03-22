classdef ThreeAxisForceTorqueContactSensor < iDynTree.LinkSensor
  methods
    function self = ThreeAxisForceTorqueContactSensor(varargin)
      self@iDynTree.LinkSensor(iDynTreeSwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1387, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1388, self);
        self.SwigClear();
      end
    end
    function varargout = setName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1389, self, varargin{:});
    end
    function varargout = setLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1390, self, varargin{:});
    end
    function varargout = setParentLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1391, self, varargin{:});
    end
    function varargout = setParentLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1392, self, varargin{:});
    end
    function varargout = getName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1393, self, varargin{:});
    end
    function varargout = getSensorType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1394, self, varargin{:});
    end
    function varargout = getParentLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1395, self, varargin{:});
    end
    function varargout = getParentLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1396, self, varargin{:});
    end
    function varargout = getLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1397, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1398, self, varargin{:});
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1399, self, varargin{:});
    end
    function varargout = updateIndices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1400, self, varargin{:});
    end
    function varargout = setLoadCellLocations(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1401, self, varargin{:});
    end
    function varargout = getLoadCellLocations(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1402, self, varargin{:});
    end
    function varargout = computeThreeAxisForceTorqueFromLoadCellMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1403, self, varargin{:});
    end
    function varargout = computeCenterOfPressureFromLoadCellMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1404, self, varargin{:});
    end
  end
  methods(Static)
  end
end
