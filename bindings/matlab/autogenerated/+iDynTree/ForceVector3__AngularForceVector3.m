classdef ForceVector3__AngularForceVector3 < iDynTree.GeomVector3__AngularForceVector3
  methods
    function self = ForceVector3__AngularForceVector3(varargin)
      self@iDynTree.GeomVector3__AngularForceVector3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(552, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(553, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
