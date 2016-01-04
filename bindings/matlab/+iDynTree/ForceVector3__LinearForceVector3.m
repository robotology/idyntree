classdef ForceVector3__LinearForceVector3 < iDynTree.GeomVector3__LinearForceVector3
  methods
    function self = ForceVector3__LinearForceVector3(varargin)
      self@iDynTree.GeomVector3__LinearForceVector3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(293, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(294, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
