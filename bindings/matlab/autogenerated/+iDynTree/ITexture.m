classdef ITexture < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1886, self);
        self.SwigClear();
      end
    end
    function varargout = environment(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1887, self, varargin{:});
    end
    function varargout = getPixelColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1888, self, varargin{:});
    end
    function varargout = getPixels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1889, self, varargin{:});
    end
    function self = ITexture(varargin)
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
