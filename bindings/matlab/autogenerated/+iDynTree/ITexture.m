classdef ITexture < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(2048, self);
        self.SwigClear();
      end
    end
    function varargout = environment(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2049, self, varargin{:});
    end
    function varargout = getPixelColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2050, self, varargin{:});
    end
    function varargout = getPixels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2051, self, varargin{:});
    end
    function varargout = drawToFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2052, self, varargin{:});
    end
    function varargout = enableDraw(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2053, self, varargin{:});
    end
    function varargout = width(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2054, self, varargin{:});
    end
    function varargout = height(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2055, self, varargin{:});
    end
    function varargout = setSubDrawArea(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2056, self, varargin{:});
    end
    function self = ITexture(varargin)
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
