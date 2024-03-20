classdef SparseMatrixColMajor < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = SparseMatrixColMajor(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(244, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(245, self);
        self.SwigClear();
      end
    end
    function varargout = numberOfNonZeros(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(246, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(247, self, varargin{:});
    end
    function varargout = reserve(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(248, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(249, self, varargin{:});
    end
    function varargout = setFromConstTriplets(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(250, self, varargin{:});
    end
    function varargout = setFromTriplets(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(251, self, varargin{:});
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(253, self, varargin{:});
    end
    function varargout = getValue(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(254, self, varargin{:});
    end
    function varargout = setValue(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(255, self, varargin{:});
    end
    function varargout = rows(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(256, self, varargin{:});
    end
    function varargout = columns(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(257, self, varargin{:});
    end
    function varargout = description(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(258, self, varargin{:});
    end
    function varargout = toMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(259, self, varargin{:});
    end
    function varargout = toMatlabDense(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(260, self, varargin{:});
    end
    function varargout = fromMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(261, self, varargin{:});
    end
  end
  methods(Static)
    function varargout = sparseMatrixFromTriplets(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(252, varargin{:});
    end
  end
end
