classdef ITexturesHandler < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(2067, self);
        self.SwigClear();
      end
    end
    function varargout = add(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2068, self, varargin{:});
    end
    function varargout = get(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2069, self, varargin{:});
    end
    function self = ITexturesHandler(varargin)
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
