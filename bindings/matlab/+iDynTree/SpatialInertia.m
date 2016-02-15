classdef SpatialInertia < iDynTree.SpatialInertiaRaw
  methods
    function self = SpatialInertia(varargin)
      self@iDynTree.SpatialInertiaRaw(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(446, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = asMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(448, self, varargin{:});
    end
    function varargout = plus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(449, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(450, self, varargin{:});
    end
    function varargout = biasWrench(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(451, self, varargin{:});
    end
    function varargout = biasWrenchDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(452, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(454, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = combine(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(447, varargin{:});
    end
    function varargout = Zero(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(453, varargin{:});
    end
  end
end
