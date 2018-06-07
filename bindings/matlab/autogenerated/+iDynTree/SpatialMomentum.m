classdef SpatialMomentum < iDynTree.SpatialForceVector
  methods
    function self = SpatialMomentum(varargin)
      self@iDynTree.SpatialForceVector(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(602, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = plus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(603, self, varargin{:});
    end
    function varargout = minus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(604, self, varargin{:});
    end
    function varargout = uminus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(605, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(606, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
