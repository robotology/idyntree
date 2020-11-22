classdef ArticulatedBodyInertia < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = ArticulatedBodyInertia(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(585, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = getLinearLinearSubmatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(586, self, varargin{:});
    end
    function varargout = getLinearAngularSubmatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(587, self, varargin{:});
    end
    function varargout = getAngularAngularSubmatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(588, self, varargin{:});
    end
    function varargout = applyInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(590, self, varargin{:});
    end
    function varargout = asMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(591, self, varargin{:});
    end
    function varargout = getInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(592, self, varargin{:});
    end
    function varargout = plus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(593, self, varargin{:});
    end
    function varargout = minus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(594, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(595, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(596, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(599, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = combine(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(589, varargin{:});
    end
    function varargout = ABADyadHelper(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(597, varargin{:});
    end
    function varargout = ABADyadHelperLin(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(598, varargin{:});
    end
  end
end
