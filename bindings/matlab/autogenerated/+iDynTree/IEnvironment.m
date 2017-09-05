classdef IEnvironment < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1658, self);
        self.swigPtr=[];
      end
    end
    function varargout = getElements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1659, self, varargin{:});
    end
    function varargout = setElementVisibility(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1660, self, varargin{:});
    end
    function varargout = setBackgroundColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1661, self, varargin{:});
    end
    function varargout = setAmbientLight(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1662, self, varargin{:});
    end
    function varargout = getLights(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1663, self, varargin{:});
    end
    function varargout = addLight(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1664, self, varargin{:});
    end
    function varargout = lightViz(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1665, self, varargin{:});
    end
    function varargout = removeLight(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1666, self, varargin{:});
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
