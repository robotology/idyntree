classdef ILight < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(2032, self);
        self.SwigClear();
      end
    end
    function varargout = getName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2033, self, varargin{:});
    end
    function varargout = setType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2034, self, varargin{:});
    end
    function varargout = getType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2035, self, varargin{:});
    end
    function varargout = setPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2036, self, varargin{:});
    end
    function varargout = getPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2037, self, varargin{:});
    end
    function varargout = setDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2038, self, varargin{:});
    end
    function varargout = getDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2039, self, varargin{:});
    end
    function varargout = setAmbientColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2040, self, varargin{:});
    end
    function varargout = getAmbientColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2041, self, varargin{:});
    end
    function varargout = setSpecularColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2042, self, varargin{:});
    end
    function varargout = getSpecularColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2043, self, varargin{:});
    end
    function varargout = setDiffuseColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2044, self, varargin{:});
    end
    function varargout = getDiffuseColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(2045, self, varargin{:});
    end
    function self = ILight(varargin)
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
