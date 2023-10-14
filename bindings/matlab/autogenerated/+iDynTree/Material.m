classdef Material < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = Material(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1052, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = name(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1053, self, varargin{:});
    end
    function varargout = hasColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1054, self, varargin{:});
    end
    function varargout = color(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1055, self, varargin{:});
    end
    function varargout = setColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1056, self, varargin{:});
    end
    function varargout = hasTexture(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1057, self, varargin{:});
    end
    function varargout = texture(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1058, self, varargin{:});
    end
    function varargout = setTexture(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1059, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1060, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
