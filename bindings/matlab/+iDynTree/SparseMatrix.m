classdef SparseMatrix < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = SparseMatrix(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(123, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(124, self);
        self.swigPtr=[];
      end
    end
    function varargout = numberOfNonZeros(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(125, self, varargin{:});
    end
    function varargout = nonZeroElementsForRowAtIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(126, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(127, self, varargin{:});
    end
    function varargout = reserve(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(128, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(129, self, varargin{:});
    end
    function varargout = setFromConstTriplets(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(130, self, varargin{:});
    end
    function varargout = setFromTriplets(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(131, self, varargin{:});
    end
    function varargout = getValue(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(132, self, varargin{:});
    end
    function varargout = setValue(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(133, self, varargin{:});
    end
    function varargout = rows(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(134, self, varargin{:});
    end
    function varargout = columns(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(135, self, varargin{:});
    end
    function varargout = description(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(136, self, varargin{:});
    end
    function varargout = convertFromColumnMajor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(137, self, varargin{:});
    end
    function varargout = toMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(138, self, varargin{:});
    end
    function varargout = toMatlabDense(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(139, self, varargin{:});
    end
    function varargout = fromMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(140, self, varargin{:});
    end
  end
  methods(Static)
    function v = RowMajor()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = iDynTreeMEX(0, 0);
      end
      v = vInitialized;
    end
    function v = ColumnMajor()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = iDynTreeMEX(0, 1);
      end
      v = vInitialized;
    end
  end
end
