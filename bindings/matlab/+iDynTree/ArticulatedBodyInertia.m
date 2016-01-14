classdef ArticulatedBodyInertia < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = ArticulatedBodyInertia(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(447, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = getLinearLinearSubmatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(448, self, varargin{:});
    end
    function varargout = getLinearAngularSubmatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(449, self, varargin{:});
    end
    function varargout = getAngularAngularSubmatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(450, self, varargin{:});
    end
    function varargout = applyInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(452, self, varargin{:});
    end
    function varargout = asMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(453, self, varargin{:});
    end
    function varargout = getInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(454, self, varargin{:});
    end
    function varargout = plus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(455, self, varargin{:});
    end
    function varargout = minus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(456, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(457, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(458, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(461, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = combine(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(451, varargin{:});
    end
    function varargout = ABADyadHelper(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(459, varargin{:});
    end
    function varargout = ABADyadHelperLin(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(460, varargin{:});
    end
  end
end
