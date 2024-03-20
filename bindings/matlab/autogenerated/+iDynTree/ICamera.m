classdef ICamera < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1947, self);
        self.SwigClear();
      end
    end
    function varargout = setPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1948, self, varargin{:});
    end
    function varargout = setTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1949, self, varargin{:});
    end
    function varargout = getPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1950, self, varargin{:});
    end
    function varargout = getTarget(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1951, self, varargin{:});
    end
    function varargout = setUpVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1952, self, varargin{:});
    end
    function varargout = animator(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1953, self, varargin{:});
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
