classdef VectorDynSize < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = VectorDynSize(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(20, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(21, self);
        self.swigPtr=[];
      end
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(22, self, varargin{:});
    end
    function varargout = getVal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(23, self, varargin{:});
    end
    function varargout = setVal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(24, self, varargin{:});
    end
    function varargout = size(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(25, self, varargin{:});
    end
    function varargout = data(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(26, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(27, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(28, self, varargin{:});
    end
    function varargout = fillBuffer(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(29, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(30, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(31, self, varargin{:});
    end
    function varargout = toMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(32, self, varargin{:});
    end
    function varargout = fromMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(33, self, varargin{:});
    end
  end
  methods(Static)
  end
end
