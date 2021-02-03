classdef ILight < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1824, self);
        self.SwigClear();
      end
    end
    function varargout = getName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1825, self, varargin{:});
    end
    function varargout = setType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1826, self, varargin{:});
    end
    function varargout = getType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1827, self, varargin{:});
    end
    function varargout = setPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1828, self, varargin{:});
    end
    function varargout = getPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1829, self, varargin{:});
    end
    function varargout = setDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1830, self, varargin{:});
    end
    function varargout = getDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1831, self, varargin{:});
    end
    function varargout = setAmbientColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1832, self, varargin{:});
    end
    function varargout = getAmbientColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1833, self, varargin{:});
    end
    function varargout = setSpecularColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1834, self, varargin{:});
    end
    function varargout = getSpecularColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1835, self, varargin{:});
    end
    function varargout = setDiffuseColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1836, self, varargin{:});
    end
    function varargout = getDiffuseColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1837, self, varargin{:});
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
