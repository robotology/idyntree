classdef IEnvironment < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1920, self);
        self.SwigClear();
      end
    end
    function varargout = getElements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1921, self, varargin{:});
    end
    function varargout = setElementVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1922, self, varargin{:});
    end
    function varargout = setBackgroundColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1923, self, varargin{:});
    end
    function varargout = setAmbientLight(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1924, self, varargin{:});
    end
    function varargout = getLights(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1925, self, varargin{:});
    end
    function varargout = addLight(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1926, self, varargin{:});
    end
    function varargout = lightViz(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1927, self, varargin{:});
    end
    function varargout = removeLight(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1928, self, varargin{:});
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
