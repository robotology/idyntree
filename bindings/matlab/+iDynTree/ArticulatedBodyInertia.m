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
        tmp = iDynTreeMEX(603, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = getLinearLinearSubmatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(604, self, varargin{:});
    end
    function varargout = getLinearAngularSubmatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(605, self, varargin{:});
    end
    function varargout = getAngularAngularSubmatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(606, self, varargin{:});
    end
    function varargout = applyInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(608, self, varargin{:});
    end
    function varargout = asMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(609, self, varargin{:});
    end
    function varargout = getInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(610, self, varargin{:});
    end
    function varargout = plus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(611, self, varargin{:});
    end
    function varargout = minus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(612, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(613, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(614, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(617, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = combine(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(607, varargin{:});
    end
    function varargout = ABADyadHelper(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(615, varargin{:});
    end
    function varargout = ABADyadHelperLin(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(616, varargin{:});
    end
  end
end
