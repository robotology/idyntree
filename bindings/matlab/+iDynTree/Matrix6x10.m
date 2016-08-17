classdef Matrix6x10 < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = Matrix6x10(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(159, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(160, self, varargin{:});
    end
    function varargout = getVal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(161, self, varargin{:});
    end
    function varargout = setVal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(162, self, varargin{:});
    end
    function varargout = rows(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(163, self, varargin{:});
    end
    function varargout = cols(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(164, self, varargin{:});
    end
    function varargout = data(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(165, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(166, self, varargin{:});
    end
    function varargout = fillRowMajorBuffer(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(167, self, varargin{:});
    end
    function varargout = fillColMajorBuffer(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(168, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(169, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(170, self, varargin{:});
    end
    function varargout = toMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(171, self, varargin{:});
    end
    function varargout = fromMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(172, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(173, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
