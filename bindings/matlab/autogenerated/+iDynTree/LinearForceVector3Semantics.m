classdef LinearForceVector3Semantics < iDynTree.ForceVector3Semantics__LinearForceVector3Semantics
  methods
    function self = LinearForceVector3Semantics(varargin)
      self@iDynTree.ForceVector3Semantics__LinearForceVector3Semantics(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(566, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(567, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
