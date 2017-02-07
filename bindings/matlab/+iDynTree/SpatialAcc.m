classdef SpatialAcc < iDynTree.SpatialMotionVector
  methods
    function self = SpatialAcc(varargin)
      self@iDynTree.SpatialMotionVector(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(563, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = plus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(564, self, varargin{:});
    end
    function varargout = minus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(565, self, varargin{:});
    end
    function varargout = uminus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(566, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(567, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
