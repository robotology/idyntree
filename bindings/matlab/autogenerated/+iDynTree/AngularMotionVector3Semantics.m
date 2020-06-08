classdef AngularMotionVector3Semantics < iDynTree.GeomVector3Semantics__AngularMotionVector3Semantics
  methods
    function self = AngularMotionVector3Semantics(varargin)
      self@iDynTree.GeomVector3Semantics__AngularMotionVector3Semantics(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(561, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(562, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
