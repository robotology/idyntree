classdef LinearForceVector3 < iDynTree.ForceVector3__LinearForceVector3
  methods
    function self = LinearForceVector3(varargin)
      self@iDynTree.ForceVector3__LinearForceVector3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(568, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(569, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
