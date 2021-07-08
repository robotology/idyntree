classdef ITexture < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1892, self);
        self.SwigClear();
      end
    end
    function varargout = environment(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1893, self, varargin{:});
    end
    function varargout = getPixelColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1894, self, varargin{:});
    end
    function varargout = getPixels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1895, self, varargin{:});
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
