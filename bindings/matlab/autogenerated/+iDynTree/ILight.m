classdef ILight < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1625, self);
        self.swigPtr=[];
      end
    end
    function varargout = getName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1626, self, varargin{:});
    end
    function varargout = setType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1627, self, varargin{:});
    end
    function varargout = getType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1628, self, varargin{:});
    end
    function varargout = setPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1629, self, varargin{:});
    end
    function varargout = getPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1630, self, varargin{:});
    end
    function varargout = setDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1631, self, varargin{:});
    end
    function varargout = getDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1632, self, varargin{:});
    end
    function varargout = setAmbientColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1633, self, varargin{:});
    end
    function varargout = getAmbientColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1634, self, varargin{:});
    end
    function varargout = setSpecularColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1635, self, varargin{:});
    end
    function varargout = getSpecularColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1636, self, varargin{:});
    end
    function varargout = setDiffuseColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1637, self, varargin{:});
    end
    function varargout = getDiffuseColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1638, self, varargin{:});
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
