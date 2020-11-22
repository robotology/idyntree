classdef SpatialMotionVector < iDynTree.SpatialMotionVectorBase
  methods
    function self = SpatialMotionVector(varargin)
      self@iDynTree.SpatialMotionVectorBase(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(491, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(492, self, varargin{:});
    end
    function varargout = cross(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(493, self, varargin{:});
    end
    function varargout = asCrossProductMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(494, self, varargin{:});
    end
    function varargout = asCrossProductMatrixWrench(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(495, self, varargin{:});
    end
    function varargout = exp(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(496, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(497, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
