classdef IEnvironment < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1503, self);
        self.swigPtr=[];
      end
    end
    function varargout = getElements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1504, self, varargin{:});
    end
    function varargout = setElementVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1505, self, varargin{:});
    end
    function varargout = setBackgroundColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1506, self, varargin{:});
    end
    function varargout = setAmbientLight(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1507, self, varargin{:});
    end
    function varargout = getLights(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1508, self, varargin{:});
    end
    function varargout = addLight(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1509, self, varargin{:});
    end
    function varargout = lightViz(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1510, self, varargin{:});
    end
    function varargout = removeLight(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1511, self, varargin{:});
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
