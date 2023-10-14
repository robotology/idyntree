classdef SpatialMomentum < iDynTree.SpatialForceVector
  methods
    function self = SpatialMomentum(varargin)
      self@iDynTree.SpatialForceVector(iDynTreeSwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(578, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = plus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(579, self, varargin{:});
    end
    function varargout = minus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(580, self, varargin{:});
    end
    function varargout = uminus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(581, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(582, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
