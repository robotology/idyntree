classdef SpatialForceVector < iDynTree.SpatialForceVectorBase
  methods
    function self = SpatialForceVector(varargin)
      self@iDynTree.SpatialForceVectorBase(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(487, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(488, self);
        self.swigPtr=[];
      end
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(489, self, varargin{:});
    end
  end
  methods(Static)
  end
end
