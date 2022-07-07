classdef ICamera < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1846, self);
        self.SwigClear();
      end
    end
    function varargout = setPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1847, self, varargin{:});
    end
    function varargout = setTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1848, self, varargin{:});
    end
    function varargout = getPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1849, self, varargin{:});
    end
    function varargout = getTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1850, self, varargin{:});
    end
    function varargout = setUpVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1851, self, varargin{:});
    end
    function varargout = animator(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1852, self, varargin{:});
    end
    function self = ICamera(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
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
