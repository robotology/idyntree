classdef IEnvironment < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1883, self);
        self.SwigClear();
      end
    end
    function varargout = getElements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1884, self, varargin{:});
    end
    function varargout = setElementVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1885, self, varargin{:});
    end
    function varargout = setBackgroundColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1886, self, varargin{:});
    end
    function varargout = setFloorGridColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1887, self, varargin{:});
    end
    function varargout = setAmbientLight(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1888, self, varargin{:});
    end
    function varargout = getLights(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1889, self, varargin{:});
    end
    function varargout = addLight(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1890, self, varargin{:});
    end
    function varargout = lightViz(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1891, self, varargin{:});
    end
    function varargout = removeLight(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1892, self, varargin{:});
    end
    function self = IEnvironment(varargin)
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
