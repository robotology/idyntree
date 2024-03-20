classdef SparseMatrixRowMajor < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = SparseMatrixRowMajor(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(226, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(227, self);
        self.SwigClear();
      end
    end
    function varargout = numberOfNonZeros(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(228, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(229, self, varargin{:});
    end
    function varargout = reserve(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(230, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(231, self, varargin{:});
    end
    function varargout = setFromConstTriplets(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(232, self, varargin{:});
    end
    function varargout = setFromTriplets(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(233, self, varargin{:});
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(235, self, varargin{:});
    end
    function varargout = getValue(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(236, self, varargin{:});
    end
    function varargout = setValue(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(237, self, varargin{:});
    end
    function varargout = rows(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(238, self, varargin{:});
    end
    function varargout = columns(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(239, self, varargin{:});
    end
    function varargout = description(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(240, self, varargin{:});
    end
    function varargout = toMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(241, self, varargin{:});
    end
    function varargout = toMatlabDense(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(242, self, varargin{:});
    end
    function varargout = fromMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(243, self, varargin{:});
    end
  end
  methods(Static)
    function varargout = sparseMatrixFromTriplets(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(234, varargin{:});
    end
  end
end
