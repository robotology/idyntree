classdef Vector4 < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = Vector4(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(407, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(408, self, varargin{:});
    end
    function varargout = brace(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(409, self, varargin{:});
    end
    function varargout = getVal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(410, self, varargin{:});
    end
    function varargout = setVal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(411, self, varargin{:});
    end
    function varargout = cbegin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(412, self, varargin{:});
    end
    function varargout = cend(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(413, self, varargin{:});
    end
    function varargout = begin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(414, self, varargin{:});
    end
    function varargout = end(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(415, self, varargin{:});
    end
    function varargout = size(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(416, self, varargin{:});
    end
    function varargout = data(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(417, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(418, self, varargin{:});
    end
    function varargout = fillBuffer(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(419, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(420, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(421, self, varargin{:});
    end
    function varargout = toMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(422, self, varargin{:});
    end
    function varargout = fromMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(423, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(424, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
