classdef MotionVector3__AngularMotionVector3 < iDynTree.GeomVector3__AngularMotionVector3
  methods
    function self = MotionVector3__AngularMotionVector3(varargin)
      self@iDynTree.GeomVector3__AngularMotionVector3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(547, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = cross(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(548, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(549, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
