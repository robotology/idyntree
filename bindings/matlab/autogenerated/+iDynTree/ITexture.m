classdef ITexture < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1991, self);
        self.SwigClear();
      end
    end
    function varargout = environment(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1992, self, varargin{:});
    end
    function varargout = getPixelColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1993, self, varargin{:});
    end
    function varargout = getPixels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1994, self, varargin{:});
    end
    function varargout = drawToFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1995, self, varargin{:});
    end
    function varargout = enableDraw(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1996, self, varargin{:});
    end
    function varargout = width(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1997, self, varargin{:});
    end
    function varargout = height(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1998, self, varargin{:});
    end
    function varargout = setSubDrawArea(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1999, self, varargin{:});
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
