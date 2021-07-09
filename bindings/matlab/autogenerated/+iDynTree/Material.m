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
        tmp = iDynTreeMEX(979, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = name(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(980, self, varargin{:});
    end
    function varargout = hasColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(981, self, varargin{:});
    end
    function varargout = color(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(982, self, varargin{:});
    end
    function varargout = setColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(983, self, varargin{:});
    end
    function varargout = hasTexture(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(984, self, varargin{:});
    end
    function varargout = texture(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(985, self, varargin{:});
    end
    function varargout = setTexture(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(986, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(987, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
