classdef AccelerometerSensor < iDynTree.LinkSensor
  methods
    function self = AccelerometerSensor(varargin)
      self@iDynTree.LinkSensor(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1316, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1317, self);
        self.SwigClear();
      end
    end
    function varargout = setName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1318, self, varargin{:});
    end
    function varargout = setLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1319, self, varargin{:});
    end
    function varargout = setParentLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1320, self, varargin{:});
    end
    function varargout = setParentLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1321, self, varargin{:});
    end
    function varargout = getName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1322, self, varargin{:});
    end
    function varargout = getSensorType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1323, self, varargin{:});
    end
    function varargout = getParentLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1324, self, varargin{:});
    end
    function varargout = getParentLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1325, self, varargin{:});
    end
    function varargout = getLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1326, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1327, self, varargin{:});
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1328, self, varargin{:});
    end
    function varargout = updateIndices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1329, self, varargin{:});
    end
    function varargout = predictMeasurement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1330, self, varargin{:});
    end
  end
  methods(Static)
  end
end
