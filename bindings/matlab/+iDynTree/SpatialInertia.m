classdef SpatialInertia < iDynTree.SpatialInertiaRaw
  methods
    function self = SpatialInertia(varargin)
      self@iDynTree.SpatialInertiaRaw(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(586, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = asMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(588, self, varargin{:});
    end
    function varargout = applyInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(589, self, varargin{:});
    end
    function varargout = getInverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(590, self, varargin{:});
    end
    function varargout = plus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(591, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(592, self, varargin{:});
    end
    function varargout = biasWrench(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(593, self, varargin{:});
    end
    function varargout = biasWrenchDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(594, self, varargin{:});
    end
    function varargout = asVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(596, self, varargin{:});
    end
    function varargout = fromVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(597, self, varargin{:});
    end
    function varargout = isPhysicallyConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(598, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(602, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = combine(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(587, varargin{:});
    end
    function varargout = Zero(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(595, varargin{:});
    end
    function varargout = momentumRegressor(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(599, varargin{:});
    end
    function varargout = momentumDerivativeRegressor(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(600, varargin{:});
    end
    function varargout = momentumDerivativeSlotineLiRegressor(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(601, varargin{:});
    end
  end
end
