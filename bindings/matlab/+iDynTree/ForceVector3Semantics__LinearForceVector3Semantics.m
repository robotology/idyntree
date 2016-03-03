classdef ForceVector3Semantics__LinearForceVector3Semantics < iDynTree.GeomVector3Semantics__LinearForceVector3Semantics
  methods
    function self = ForceVector3Semantics__LinearForceVector3Semantics(varargin)
      self@iDynTree.GeomVector3Semantics__LinearForceVector3Semantics(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(344, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(347, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(345, varargin{:});
    end
    function varargout = inverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(346, varargin{:});
    end
  end
end
