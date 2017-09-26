classdef SparseMatrixColMajor < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = SparseMatrixColMajor(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(139, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(140, self);
        self.swigPtr=[];
      end
    end
    function varargout = numberOfNonZeros(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(141, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(142, self, varargin{:});
    end
    function varargout = reserve(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(143, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(144, self, varargin{:});
    end
    function varargout = setFromConstTriplets(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(145, self, varargin{:});
    end
    function varargout = setFromTriplets(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(146, self, varargin{:});
    end
    function varargout = getValue(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(147, self, varargin{:});
    end
    function varargout = setValue(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(148, self, varargin{:});
    end
    function varargout = rows(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(149, self, varargin{:});
    end
    function varargout = columns(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(150, self, varargin{:});
    end
    function varargout = description(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(151, self, varargin{:});
    end
    function varargout = toMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(152, self, varargin{:});
    end
    function varargout = toMatlabDense(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(153, self, varargin{:});
    end
    function varargout = fromMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(154, self, varargin{:});
    end
  end
  methods(Static)
  end
end
