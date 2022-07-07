classdef ICameraAnimator < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = enableMouseControl(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1838, self, varargin{:});
    end
    function varargout = getMoveSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1839, self, varargin{:});
    end
    function varargout = setMoveSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1840, self, varargin{:});
    end
    function varargout = getRotateSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1841, self, varargin{:});
    end
    function varargout = setRotateSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1842, self, varargin{:});
    end
    function varargout = getZoomSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1843, self, varargin{:});
    end
    function varargout = setZoomSpeed(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1844, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1845, self);
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
