classdef SpatialInertia < iDynTree.SpatialInertiaRaw
  methods
    function self = SpatialInertia(varargin)
      self@iDynTree.SpatialInertiaRaw(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(548, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = asMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(550, self, varargin{:});
    end
    function varargout = plus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(551, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(552, self, varargin{:});
    end
    function varargout = biasWrench(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(553, self, varargin{:});
    end
    function varargout = biasWrenchDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(554, self, varargin{:});
    end
    function varargout = asVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(556, self, varargin{:});
    end
    function varargout = fromVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(557, self, varargin{:});
    end
    function varargout = isPhysicallyConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(558, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(562, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = combine(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(549, varargin{:});
    end
    function varargout = Zero(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(555, varargin{:});
    end
    function varargout = momentumRegressor(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(559, varargin{:});
    end
    function varargout = momentumDerivativeRegressor(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(560, varargin{:});
    end
    function varargout = momentumDerivativeSlotineLiRegressor(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(561, varargin{:});
    end
  end
end
