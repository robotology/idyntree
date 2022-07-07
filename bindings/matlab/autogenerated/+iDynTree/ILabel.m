classdef ILabel < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1901, self);
        self.SwigClear();
      end
    end
    function varargout = setText(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1902, self, varargin{:});
    end
    function varargout = getText(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1903, self, varargin{:});
    end
    function varargout = setSize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1904, self, varargin{:});
    end
    function varargout = width(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1905, self, varargin{:});
    end
    function varargout = height(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1906, self, varargin{:});
    end
    function varargout = setPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1907, self, varargin{:});
    end
    function varargout = getPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1908, self, varargin{:});
    end
    function varargout = setColor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1909, self, varargin{:});
    end
    function varargout = setVisible(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1910, self, varargin{:});
    end
    function self = ILabel(varargin)
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
