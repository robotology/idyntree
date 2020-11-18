classdef Vector3 < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = Vector3(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(314, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(315, self, varargin{:});
    end
    function varargout = brace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(316, self, varargin{:});
    end
    function varargout = getVal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(317, self, varargin{:});
    end
    function varargout = setVal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(318, self, varargin{:});
    end
    function varargout = cbegin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(319, self, varargin{:});
    end
    function varargout = cend(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(320, self, varargin{:});
    end
    function varargout = begin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(321, self, varargin{:});
    end
    function varargout = end(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(322, self, varargin{:});
    end
    function varargout = size(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(323, self, varargin{:});
    end
    function varargout = data(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(324, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(325, self, varargin{:});
    end
    function varargout = fillBuffer(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(326, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(327, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(328, self, varargin{:});
    end
    function varargout = toMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(329, self, varargin{:});
    end
    function varargout = fromMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(330, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(331, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
