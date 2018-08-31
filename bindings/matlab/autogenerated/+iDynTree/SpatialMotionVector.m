classdef SpatialMotionVector < iDynTree.SpatialMotionVectorBase
  methods
    function self = SpatialMotionVector(varargin)
      self@iDynTree.SpatialMotionVectorBase(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(584, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(585, self, varargin{:});
    end
    function varargout = cross(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(586, self, varargin{:});
    end
    function varargout = asCrossProductMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(587, self, varargin{:});
    end
    function varargout = asCrossProductMatrixWrench(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(588, self, varargin{:});
    end
    function varargout = exp(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(589, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(590, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
