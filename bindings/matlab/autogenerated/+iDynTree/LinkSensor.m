classdef LinkSensor < iDynTree.Sensor
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1236, self);
        self.SwigClear();
      end
    end
    function varargout = getParentLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1237, self, varargin{:});
    end
    function varargout = getParentLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1238, self, varargin{:});
    end
    function varargout = getLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1239, self, varargin{:});
    end
    function varargout = setParentLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1240, self, varargin{:});
    end
    function varargout = setParentLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1241, self, varargin{:});
    end
    function varargout = setLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1242, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1243, self, varargin{:});
    end
    function self = LinkSensor(varargin)
      self@iDynTree.Sensor(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end
