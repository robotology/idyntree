classdef IEnvironment < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1838, self);
        self.SwigClear();
      end
    end
    function varargout = getElements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1839, self, varargin{:});
    end
    function varargout = setElementVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1840, self, varargin{:});
    end
    function varargout = setBackgroundColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1841, self, varargin{:});
    end
    function varargout = setFloorGridColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1842, self, varargin{:});
    end
    function varargout = setAmbientLight(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1843, self, varargin{:});
    end
    function varargout = getLights(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1844, self, varargin{:});
    end
    function varargout = addLight(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1845, self, varargin{:});
    end
    function varargout = lightViz(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1846, self, varargin{:});
    end
    function varargout = removeLight(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1847, self, varargin{:});
    end
    function self = IEnvironment(varargin)
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
