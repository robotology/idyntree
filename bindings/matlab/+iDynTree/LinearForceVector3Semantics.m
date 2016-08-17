classdef LinearForceVector3Semantics < iDynTree.ForceVector3Semantics__LinearForceVector3Semantics
  methods
    function self = LinearForceVector3Semantics(varargin)
      self@iDynTree.ForceVector3Semantics__LinearForceVector3Semantics(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(407, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(408, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
