classdef ICameraAnimator < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = enableMouseControl(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1939, self, varargin{:});
    end
    function varargout = getMoveSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1940, self, varargin{:});
    end
    function varargout = setMoveSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1941, self, varargin{:});
    end
    function varargout = getRotateSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1942, self, varargin{:});
    end
    function varargout = setRotateSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1943, self, varargin{:});
    end
    function varargout = getZoomSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1944, self, varargin{:});
    end
    function varargout = setZoomSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1945, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1946, self);
        self.SwigClear();
      end
    end
    function self = ICameraAnimator(varargin)
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
