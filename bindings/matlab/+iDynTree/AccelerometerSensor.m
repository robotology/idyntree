classdef AccelerometerSensor < iDynTree.LinkSensor
  methods
    function self = AccelerometerSensor(varargin)
      self@iDynTree.LinkSensor(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1201, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1202, self);
        self.swigPtr=[];
      end
    end
    function varargout = setName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1203, self, varargin{:});
    end
    function varargout = setLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1204, self, varargin{:});
    end
    function varargout = setParentLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1205, self, varargin{:});
    end
    function varargout = setParentLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1206, self, varargin{:});
    end
    function varargout = getName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1207, self, varargin{:});
    end
    function varargout = getSensorType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1208, self, varargin{:});
    end
    function varargout = getParentLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1209, self, varargin{:});
    end
    function varargout = getParentLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1210, self, varargin{:});
    end
    function varargout = getLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1211, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1212, self, varargin{:});
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1213, self, varargin{:});
    end
    function varargout = updateIndeces(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1214, self, varargin{:});
    end
    function varargout = predictMeasurement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1215, self, varargin{:});
    end
  end
  methods(Static)
  end
end
