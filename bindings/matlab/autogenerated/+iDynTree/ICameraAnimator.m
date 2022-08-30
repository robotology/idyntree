classdef ICameraAnimator < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = enableMouseControl(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1851, self, varargin{:});
    end
    function varargout = getMoveSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1852, self, varargin{:});
    end
    function varargout = setMoveSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1853, self, varargin{:});
    end
    function varargout = getRotateSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1854, self, varargin{:});
    end
    function varargout = setRotateSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1855, self, varargin{:});
    end
    function varargout = getZoomSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1856, self, varargin{:});
    end
    function varargout = setZoomSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1857, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1858, self);
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
